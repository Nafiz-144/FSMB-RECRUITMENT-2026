/*
 * Smart Voltage & Over-Current Protector
 * MCU      : PIC18F4550 (20 MHz crystal, HS mode, no PLL)
 * Compiler : MPLAB XC8 v2.x   (MPLAB X IDE)
 *
 * ---------------------------------------------------------------------------
 *  PIN MAP
 *   RA0 / AN0 : mains voltage  (ZMPT101B output, centred at ~2.5 V)
 *   RA1 / AN1 : load current   (ACS712 output,   centred at ~2.5 V)
 *   RB0       : button UP   (to GND, internal pull-up)
 *   RB1       : button DOWN (to GND, internal pull-up)
 *   RB2       : button OK   (to GND, internal pull-up)
 *   RC0       : LCD RS            RC1 : LCD EN      (LCD RW -> GND)
 *   RD4..RD7  : LCD D4..D7
 *   RD0       : relay driver (via NPN transistor, HIGH = relay ON)
 *   RD1       : buzzer
 *   RD2       : green LED (relay ON)
 *   RD3       : red LED   (fault)
 *   RC6 / RC7 : UART TX / RX  -> USB-UART adapter (9600 baud, 8N1)
 *
 *  SERIAL COMMANDS (end each line with Enter)
 *   STATUS | GET | SET <name> <value> | SAVE | RESET | LOG | DEFAULTS | HELP
 *   names: OV UV OC DELAY RECON VCAL ICAL SURGE      e.g.  SET OC 6.5
 *
 *  NOT TESTED ON HARDWARE. Calibrate VCAL and ICAL against a multimeter.
 * ---------------------------------------------------------------------------
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define _XTAL_FREQ 20000000UL

/* ---- configuration bits ------------------------------------------------- */
#pragma config PLLDIV = 5, CPUDIV = OSC1_PLL2, USBDIV = 1
#pragma config FOSC = HS, FCMEN = OFF, IESO = OFF
#pragma config PWRT = ON, BOR = ON, BORV = 1, VREGEN = OFF
#pragma config WDT = ON, WDTPS = 256 /* about 1 s watchdog        */
#pragma config MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF, CCP2MX = ON
#pragma config STVREN = ON, LVP = OFF, XINST = OFF, DEBUG = OFF

/* ---- pins --------------------------------------------------------------- */
#define RELAY LATDbits.LATD0
#define BUZZER LATDbits.LATD1
#define LED_OK LATDbits.LATD2
#define LED_FAULT LATDbits.LATD3
#define PIN_UP PORTBbits.RB0
#define PIN_DN PORTBbits.RB1
#define PIN_OK PORTBbits.RB2
#define LCD_RS LATCbits.LATC0
#define LCD_EN LATCbits.LATC1

/* ---- measurement constants --------------------------------------------- */
#define N_SAMP 100              /* samples per 20 ms mains cycle        */
#define SAMPLE_TICKS 1000       /* Timer1 ticks (5 MHz) = 200 us        */
#define MAX_TRIPS 3             /* trips inside window -> lockout       */
#define TRIP_WINDOW_MS 300000UL /* 5 minutes                            */
#define STARTUP_WAIT_MS 3000UL
#define OV_HYST 10 /* volts below OV before recovery       */
#define UV_HYST 5  /* volts above UV before recovery       */

/* ---- settings ----------------------------------------------------------- */
enum
{
    S_OV,
    S_UV,
    S_OC,
    S_DELAY,
    S_RECON,
    S_VCAL,
    S_ICAL,
    S_SURGE,
    N_SET
};

typedef struct
{
    char name[6];
    uint16_t min, max, def, step;
    uint8_t dec; /* decimals shown (OC has 1)            */
} setting_t;

static const setting_t SET_TBL[N_SET] = {
    {"OV", 220, 280, 240, 1, 0},        /* V rms                           */
    {"UV", 120, 210, 180, 1, 0},        /* V rms                           */
    {"OC", 5, 200, 50, 1, 1},           /* A rms (stored x10)              */
    {"DELAY", 20, 3000, 200, 20, 0},    /* ms before a normal trip         */
    {"RECON", 5, 600, 30, 5, 0},        /* s before auto reconnect         */
    {"VCAL", 20, 300, 100, 1, 0},       /* 0.01 V per ADC count            */
    {"ICAL", 1000, 20000, 4888, 10, 0}, /* 0.001 A per ADC count x1000/... */
    {"SURGE", 300, 500, 380, 5, 0}      /* peak volts for instant trip     */
};

static uint16_t cfg[N_SET];
static uint16_t surge_cnt; /* peak limit, ADC counts (voltage)     */
static uint16_t short_cnt; /* peak limit, ADC counts (current)     */

/* ---- faults / state ----------------------------------------------------- */
typedef enum
{
    F_NONE = 0,
    F_OV,
    F_UV,
    F_OC,
    F_SURGE,
    F_SHORT
} fault_t;
static const char FAULT_NAME[6][6] = {"NONE", "OV", "UV", "OC", "SURGE", "SHORT"};

typedef enum
{
    ST_FAULT = 0,
    ST_NORMAL,
    ST_LOCKOUT
} state_t;

typedef struct
{
    uint16_t v10;     /* volts x10                            */
    uint16_t i100;    /* amps x100                            */
    fault_t hard;     /* instant fault seen while sampling    */
    uint16_t hardval; /* value for the log                    */
} meas_t;

static meas_t meas;
static state_t state = ST_FAULT;
static fault_t cur_fault = F_NONE;
static bool relay_on = false;
static bool pending = false;
static fault_t pend_fault = F_NONE;
static uint32_t pend_since = 0;
static bool healthy_run = false;
static uint32_t healthy_since = 0;
static uint32_t wait_ms = STARTUP_WAIT_MS;
static uint8_t trip_cnt = 0;
static uint32_t trip_win_start = 0;
static uint32_t buzz_until = 0;
static fault_t last_fault = F_NONE;
static uint16_t last_val = 0;
static bool defaults_used = false;

/* ---- time base and UART (interrupt driven) ------------------------------ */
static volatile uint32_t ms_ticks;
#define RX_MAX 32
static volatile char rx_buf[RX_MAX];
static volatile uint8_t rx_idx;
static volatile bool line_ready;
#define TX_SIZE 128
static volatile char tx_buf[TX_SIZE];
static volatile uint8_t tx_head, tx_tail;

void __interrupt() isr(void)
{
    if (INTCONbits.TMR0IF)
    { /* 1 ms tick                  */
        TMR0H = 0xFD;
        TMR0L = 0x8F;
        INTCONbits.TMR0IF = 0;
        ms_ticks++;
    }
    if (PIR1bits.RCIF)
    { /* receive                    */
        if (RCSTAbits.OERR)
        {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }
        char c = RCREG;
        if (!line_ready)
        {
            if (c == '\n' || c == '\r')
            {
                if (rx_idx > 0)
                {
                    rx_buf[rx_idx] = 0;
                    line_ready = true;
                }
                rx_idx = 0;
            }
            else if (rx_idx < RX_MAX - 1)
            {
                rx_buf[rx_idx++] = c;
            }
            else
            {
                rx_idx = 0; /* too long: discard          */
            }
        }
    }
    if (PIE1bits.TXIE && PIR1bits.TXIF)
    { /* transmit                   */
        if (tx_head != tx_tail)
        {
            TXREG = tx_buf[tx_tail];
            tx_tail = (uint8_t)((tx_tail + 1) & (TX_SIZE - 1));
        }
        else
        {
            PIE1bits.TXIE = 0;
        }
    }
}

static uint32_t millis(void)
{
    uint32_t t;
    uint8_t g = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    t = ms_ticks;
    INTCONbits.GIE = g;
    return t;
}

static void uart_putc(char c)
{
    uint8_t g = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    uint8_t next = (uint8_t)((tx_head + 1) & (TX_SIZE - 1));
    if (next != tx_tail)
    { /* if full the char is dropped
         so protection never stalls */
        tx_buf[tx_head] = c;
        tx_head = next;
        PIE1bits.TXIE = 1;
    }
    INTCONbits.GIE = g;
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

/* ---- small text helpers (no printf, saves flash) ------------------------ */
static char *app(char *d, const char *s)
{
    while (*s)
        *d++ = *s++;
    *d = 0;
    return d;
}

/* write v with 'dec' decimals, e.g. v=1205 dec=2 -> "12.05" */
static char *fmt_fixed(char *d, uint32_t v, uint8_t dec)
{
    char t[11];
    uint8_t n = 0;
    do
    {
        t[n++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v || n <= dec);
    while (n)
    {
        n--;
        *d++ = t[n];
        if (dec && n == dec)
            *d++ = '.';
    }
    *d = 0;
    return d;
}

static void pad16(char *s)
{
    uint8_t i = 0;
    while (s[i] && i < 16)
        i++;
    while (i < 16)
        s[i++] = ' ';
    s[16] = 0;
}

static bool parse_num(const char *s, uint8_t dec, uint16_t *out)
{
    uint32_t ip = 0, fp = 0, scale = 1;
    uint8_t fd = 0;
    bool digits = false;
    while (*s >= '0' && *s <= '9')
    {
        ip = ip * 10 + (uint8_t)(*s - '0');
        if (ip > 100000UL)
            return false;
        digits = true;
        s++;
    }
    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            if (fd < dec)
            {
                fp = fp * 10 + (uint8_t)(*s - '0');
                fd++;
            }
            digits = true;
            s++;
        }
    }
    if (!digits || *s)
        return false;
    while (fd < dec)
    {
        fp *= 10;
        fd++;
    }
    for (uint8_t i = 0; i < dec; i++)
        scale *= 10;
    uint32_t v = ip * scale + fp;
    if (v > 65535UL)
        return false;
    *out = (uint16_t)v;
    return true;
}

/* ---- EEPROM ------------------------------------------------------------- */
#define EE_MAGIC 0xA6
#define EE_CSUM (1 + 2 * N_SET)
#define EE_LF 0x20 /* last-fault record           */
#define EE_LF_MAGIC 0x3C

static uint8_t ee_read(uint8_t a)
{
    EEADR = a;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;
    return EEDATA;
}

static void ee_write(uint8_t a, uint8_t v)
{
    if (ee_read(a) == v)
        return; /* spare the EEPROM            */
    EEADR = a;
    EEDATA = v;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;
    uint8_t g = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    INTCONbits.GIE = g;
    while (EECON1bits.WR)
        CLRWDT();
    EECON1bits.WREN = 0;
    PIR2bits.EEIF = 0;
}

static uint8_t checksum_of(const uint16_t *c)
{
    uint8_t s = 0x55;
    for (uint8_t i = 0; i < N_SET; i++)
    {
        s = (uint8_t)(s + (uint8_t)(c[i] & 0xFF) + (uint8_t)(c[i] >> 8));
    }
    return s;
}

static bool validate_cfg(const uint16_t *c)
{
    for (uint8_t i = 0; i < N_SET; i++)
    {
        if (c[i] < SET_TBL[i].min || c[i] > SET_TBL[i].max)
            return false;
    }
    if (c[S_OV] < (uint16_t)(c[S_UV] + 20))
        return false; /* keep a gap    */
    return true;
}

static void settings_defaults(void)
{
    for (uint8_t i = 0; i < N_SET; i++)
        cfg[i] = SET_TBL[i].def;
}

static void recompute_limits(void)
{
    uint32_t s = (uint32_t)cfg[S_SURGE] * 100UL / cfg[S_VCAL];
    uint32_t c = (uint32_t)cfg[S_OC] * 30UL * 1000UL / cfg[S_ICAL];
    surge_cnt = (uint16_t)(s > 500 ? 500 : s);
    short_cnt = (uint16_t)(c > 500 ? 500 : c);
}

static void settings_save(void)
{
    ee_write(0, EE_MAGIC);
    for (uint8_t i = 0; i < N_SET; i++)
    {
        ee_write((uint8_t)(1 + 2 * i), (uint8_t)(cfg[i] & 0xFF));
        ee_write((uint8_t)(2 + 2 * i), (uint8_t)(cfg[i] >> 8));
    }
    ee_write(EE_CSUM, checksum_of(cfg));
}

static void settings_load(void)
{
    uint16_t tmp[N_SET];
    defaults_used = true;
    if (ee_read(0) == EE_MAGIC)
    {
        for (uint8_t i = 0; i < N_SET; i++)
        {
            tmp[i] = (uint16_t)ee_read((uint8_t)(1 + 2 * i)) | ((uint16_t)ee_read((uint8_t)(2 + 2 * i)) << 8);
        }
        if (ee_read(EE_CSUM) == checksum_of(tmp) && validate_cfg(tmp))
        {
            memcpy(cfg, tmp, sizeof cfg);
            defaults_used = false;
        }
    }
    if (defaults_used)
        settings_defaults();
    recompute_limits();

    if (ee_read(EE_LF) == EE_LF_MAGIC)
    {
        uint8_t f = ee_read(EE_LF + 1);
        if (f <= F_SHORT)
        {
            last_fault = (fault_t)f;
            last_val = (uint16_t)ee_read(EE_LF + 2) | ((uint16_t)ee_read(EE_LF + 3) << 8);
        }
    }
}

static void save_last_fault(void)
{
    ee_write(EE_LF, EE_LF_MAGIC);
    ee_write(EE_LF + 1, (uint8_t)last_fault);
    ee_write(EE_LF + 2, (uint8_t)(last_val & 0xFF));
    ee_write(EE_LF + 3, (uint8_t)(last_val >> 8));
}

/* ---- LCD 16x2, 4-bit mode ----------------------------------------------- */
static void lcd_nibble(uint8_t n)
{
    LATD = (uint8_t)((LATD & 0x0F) | (uint8_t)(n << 4));
    LCD_EN = 1;
    __delay_us(2);
    LCD_EN = 0;
    __delay_us(50);
}

static void lcd_byte(uint8_t b, bool rs)
{
    LCD_RS = rs;
    lcd_nibble(b >> 4);
    lcd_nibble(b & 0x0F);
}

static void lcd_init(void)
{
    __delay_ms(50);
    LCD_RS = 0;
    lcd_nibble(0x3);
    __delay_ms(5);
    lcd_nibble(0x3);
    __delay_us(150);
    lcd_nibble(0x3);
    lcd_nibble(0x2);
    lcd_byte(0x28, false); /* 4-bit, 2 lines              */
    lcd_byte(0x0C, false); /* display on, no cursor       */
    lcd_byte(0x06, false);
    lcd_byte(0x01, false);
    __delay_ms(2);
}

static void lcd_line(uint8_t row, const char *s16)
{
    lcd_byte((uint8_t)(row ? 0xC0 : 0x80), false);
    for (uint8_t i = 0; i < 16; i++)
        lcd_byte((uint8_t)s16[i], true);
}

/* ---- relay -------------------------------------------------------------- */
static void relay_set(bool on)
{
    relay_on = on;
    RELAY = on ? 1 : 0;
}

/* ---- measurement -------------------------------------------------------- */
static int16_t vmean = 512, imean = 512;

static uint16_t tmr1_read(void)
{
    uint16_t t = TMR1L; /* low first (RD16 = 1)        */
    t |= (uint16_t)TMR1H << 8;
    return t;
}

static uint16_t adc_read(uint8_t ch)
{
    ADCON0 = (uint8_t)((ch << 2) | 0x01); /* channel, ADC on             */
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO)
    {
    }
    return (uint16_t)(((uint16_t)ADRESH << 8) | ADRESL);
}

static uint16_t isqrt32(uint32_t x)
{
    uint32_t res = 0, bit = 1UL << 30;
    while (bit > x)
        bit >>= 2;
    while (bit)
    {
        if (x >= res + bit)
        {
            x -= res + bit;
            res = (res >> 1) + bit;
        }
        else
            res >>= 1;
        bit >>= 2;
    }
    return (uint16_t)res;
}

/* one 20 ms window: RMS of voltage and current + instant surge/short check */
static void measure(meas_t *m)
{
    int32_t sv = 0, si = 0;
    uint32_t qv = 0, qi = 0;
    uint8_t vhit = 0, ihit = 0;
    uint16_t next = tmr1_read();

    m->hard = F_NONE;
    for (uint8_t k = 0; k < N_SAMP; k++)
    {
        next += SAMPLE_TICKS;
        while ((int16_t)(tmr1_read() - next) < 0)
        {
        }

        uint16_t av = adc_read(0);
        uint16_t ai = adc_read(1);

        int16_t dv = (int16_t)av - 512;
        int16_t di = (int16_t)ai - 512;
        sv += dv;
        qv += (uint32_t)((int32_t)dv * dv);
        si += di;
        qi += (uint32_t)((int32_t)di * di);

        int16_t pv = (int16_t)av - vmean;
        if (pv < 0)
            pv = -pv;
        int16_t pi = (int16_t)ai - imean;
        if (pi < 0)
            pi = -pi;

        /* two samples in a row above the limit = real event, not noise */
        vhit = (pv > (int16_t)surge_cnt) ? (uint8_t)(vhit + 1) : 0;
        ihit = (pi > (int16_t)short_cnt) ? (uint8_t)(ihit + 1) : 0;

        if (ihit >= 2)
        {
            relay_set(false);
            m->hard = F_SHORT;
            m->hardval = (uint16_t)((uint32_t)pi * cfg[S_ICAL] / 1000UL);
            return;
        }
        if (vhit >= 2)
        {
            relay_set(false);
            m->hard = F_SURGE;
            m->hardval = (uint16_t)((uint32_t)pv * cfg[S_VCAL] / 10UL);
            return;
        }
    }

    vmean = (int16_t)(512 + sv / N_SAMP);
    imean = (int16_t)(512 + si / N_SAMP);

    uint32_t a = (uint32_t)(sv < 0 ? -sv : sv);
    uint32_t b = (uint32_t)(si < 0 ? -si : si);
    uint32_t mv = a * a / N_SAMP;
    uint32_t mi = b * b / N_SAMP;
    uint32_t varv = (qv > mv) ? (qv - mv) : 0;
    uint32_t vari = (qi > mi) ? (qi - mi) : 0;

    /* sqrt(var*16) = 4 * rms, keeps two extra bits of resolution */
    uint32_t rv4 = isqrt32(varv * 16UL / N_SAMP);
    uint32_t ri4 = isqrt32(vari * 16UL / N_SAMP);

    m->v10 = (uint16_t)(rv4 * cfg[S_VCAL] / 40UL);
    m->i100 = (uint16_t)(ri4 * cfg[S_ICAL] / 4000UL);
    if (m->i100 < 5)
        m->i100 = 0; /* ignore sensor noise < 50 mA */
}

/* ---- protection logic --------------------------------------------------- */
static void send_event(fault_t f, uint16_t val)
{
    char b[48];
    char *p = app(b, "EVENT TRIP ");
    p = app(p, FAULT_NAME[f]);
    p = app(p, " ");
    if (f == F_OC || f == F_SHORT)
    {
        p = fmt_fixed(p, val, 2);
        p = app(p, "A");
    }
    else
    {
        p = fmt_fixed(p, val, 1);
        p = app(p, "V");
    }
    app(p, "\r\n");
    uart_puts(b);
}

static void do_trip(fault_t f, uint16_t val)
{
    uint32_t now = millis();
    relay_set(false);
    state = ST_FAULT;
    cur_fault = f;
    pending = false;
    healthy_run = false;
    wait_ms = (uint32_t)cfg[S_RECON] * 1000UL;

    if (trip_cnt == 0 || (now - trip_win_start) > TRIP_WINDOW_MS)
    {
        trip_cnt = 1;
        trip_win_start = now;
    }
    else
    {
        trip_cnt++;
    }
    if (trip_cnt >= MAX_TRIPS)
        state = ST_LOCKOUT;

    buzz_until = now + 2000UL;
    send_event(f, val);
    if (state == ST_LOCKOUT)
        uart_puts("EVENT LOCKOUT (send RESET)\r\n");

    last_fault = f;
    last_val = val;
    save_last_fault(); /* relay is already off        */
}

static void clear_fault(void)
{
    trip_cnt = 0;
    pending = false;
    healthy_run = false;
    cur_fault = F_NONE;
    wait_ms = STARTUP_WAIT_MS;
    state = ST_FAULT; /* reconnect after short wait  */
}

static void protection_step(const meas_t *m)
{
    uint32_t now = millis();

    if (state == ST_NORMAL)
    {
        if (m->hard != F_NONE)
        {
            do_trip(m->hard, m->hardval);
            return;
        }

        fault_t f = F_NONE;
        uint16_t val = m->v10;
        if (m->v10 > (uint16_t)(cfg[S_OV] * 10U))
            f = F_OV;
        else if (m->v10 < (uint16_t)(cfg[S_UV] * 10U))
            f = F_UV;
        else if (m->i100 > (uint16_t)(cfg[S_OC] * 10U))
        {
            f = F_OC;
            val = m->i100;
        }

        if (f == F_NONE)
        {
            pending = false;
            return;
        }
        if (!pending || pend_fault != f)
        {
            pending = true;
            pend_fault = f;
            pend_since = now;
        }
        else if ((now - pend_since) >= cfg[S_DELAY])
        {
            do_trip(f, val);
        }
    }
    else if (state == ST_FAULT)
    {
        bool healthy = (m->hard == F_NONE) && m->v10 <= (uint16_t)((cfg[S_OV] - OV_HYST) * 10U) && m->v10 >= (uint16_t)((cfg[S_UV] + UV_HYST) * 10U);
        if (healthy)
        {
            if (!healthy_run)
            {
                healthy_run = true;
                healthy_since = now;
            }
            else if ((now - healthy_since) >= wait_ms)
            {
                relay_set(true);
                state = ST_NORMAL;
                cur_fault = F_NONE;
                pending = false;
            }
        }
        else
        {
            healthy_run = false;
        }
    }
    /* ST_LOCKOUT: stay off until RESET command or OK button */
}

/* ---- buttons ------------------------------------------------------------ */
#define EV_UP 1
#define EV_DN 2
#define EV_OK 4  /* short press, on release */
#define EV_OKL 8 /* long press (about 1 s)  */

static uint8_t repeat_btn(uint8_t *cnt, bool down)
{
    if (!down)
    {
        *cnt = 0;
        return 0;
    }
    if (*cnt < 255)
        (*cnt)++;
    if (*cnt == 2)
        return 1;
    if (*cnt >= 30)
    {
        *cnt = 25;
        return 1;
    } /* auto-repeat while held      */
    return 0;
}

static uint8_t buttons_poll(void) /* call once per loop (~25 ms) */
{
    static uint8_t cu, cd, co;
    static bool okl_sent;
    uint8_t ev = 0;

    if (repeat_btn(&cu, PIN_UP == 0))
        ev |= EV_UP;
    if (repeat_btn(&cd, PIN_DN == 0))
        ev |= EV_DN;

    if (PIN_OK == 0)
    {
        if (co < 255)
            co++;
        if (co >= 40 && !okl_sent)
        {
            okl_sent = true;
            ev |= EV_OKL;
        }
    }
    else
    {
        if (co >= 2 && !okl_sent)
            ev |= EV_OK;
        co = 0;
        okl_sent = false;
    }
    return ev;
}

/* ---- menu --------------------------------------------------------------- */
static bool menu_active;
static uint8_t menu_idx;
static uint16_t edit[N_SET];
static uint32_t menu_last;
static uint32_t msg_until;
static char msg[17];

static void show_msg(const char *s)
{
    char b[17];
    app(b, s);
    pad16(b);
    memcpy(msg, b, 17);
    msg_until = millis() + 1500UL;
}

static void menu_draw(void)
{
    char a[20], b[20], *p;
    p = app(a, "SET ");
    app(p, SET_TBL[menu_idx].name);
    pad16(a);
    p = fmt_fixed(b, edit[menu_idx], SET_TBL[menu_idx].dec);
    app(p, "  OK=next");
    pad16(b);
    lcd_line(0, a);
    lcd_line(1, b);
}

static void ui_handle(uint8_t ev)
{
    uint32_t now = millis();

    if (!menu_active)
    {
        if (ev & EV_OKL)
        {
            menu_active = true;
            menu_idx = 0;
            memcpy(edit, cfg, sizeof cfg);
            menu_last = now;
        }
        else if ((ev & EV_OK) && state == ST_LOCKOUT)
        {
            clear_fault();
            uart_puts("EVENT LOCKOUT CLEARED\r\n");
        }
        return;
    }

    if (ev)
        menu_last = now;

    if (ev & EV_UP)
    {
        int32_t v = (int32_t)edit[menu_idx] + SET_TBL[menu_idx].step;
        edit[menu_idx] = (uint16_t)(v > SET_TBL[menu_idx].max ? SET_TBL[menu_idx].max : v);
    }
    if (ev & EV_DN)
    {
        int32_t v = (int32_t)edit[menu_idx] - SET_TBL[menu_idx].step;
        edit[menu_idx] = (uint16_t)(v < SET_TBL[menu_idx].min ? SET_TBL[menu_idx].min : v);
    }
    if (ev & EV_OK)
    {
        menu_idx++;
        if (menu_idx >= N_SET)
        {
            menu_active = false;
            if (validate_cfg(edit))
            {
                memcpy(cfg, edit, sizeof cfg);
                recompute_limits();
                settings_save();
                show_msg("SETTINGS SAVED");
            }
            else
            {
                show_msg("INVALID - NOT SET");
            }
            return;
        }
    }
    if ((ev & EV_OKL) || (now - menu_last) > 10000UL)
    { /* cancel         */
        menu_active = false;
        show_msg("MENU CANCELLED");
    }
}

/* ---- main screen -------------------------------------------------------- */
static void display_update(void)
{
    char a[24], b[24], *p;

    p = app(a, "V=");
    p = fmt_fixed(p, meas.v10, 1);
    p = app(p, " I=");
    p = fmt_fixed(p, meas.i100, 2);
    app(p, "A");
    pad16(a);

    if (millis() < msg_until && msg[0])
    {
        memcpy(b, msg, 17);
    }
    else if (state == ST_NORMAL)
    {
        p = app(b, "RELAY ON");
        if (pending)
        {
            p = app(p, " ?");
            p = app(p, FAULT_NAME[pend_fault]);
        }
        pad16(b);
    }
    else if (state == ST_LOCKOUT)
    {
        app(b, "LOCKOUT OK=reset");
    }
    else
    {
        uint32_t left = wait_ms;
        if (healthy_run)
        {
            uint32_t el = millis() - healthy_since;
            left = (el >= wait_ms) ? 0 : wait_ms - el;
        }
        p = (cur_fault == F_NONE) ? app(b, "START ") : app(b, FAULT_NAME[cur_fault]);
        if (cur_fault != F_NONE)
            p = app(p, " TRIP ");
        p = app(p, "wait ");
        p = fmt_fixed(p, left / 1000UL, 0);
        app(p, "s");
        pad16(b);
    }
    lcd_line(0, a);
    lcd_line(1, b);
}

/* ---- serial commands ---------------------------------------------------- */
static int8_t find_setting(const char *n)
{
    for (uint8_t i = 0; i < N_SET; i++)
    {
        if (!strcmp(n, SET_TBL[i].name))
            return (int8_t)i;
    }
    return -1;
}

static void cmd_status(void)
{
    char b[96], *p;
    p = app(b, "V=");
    p = fmt_fixed(p, meas.v10, 1);
    p = app(p, " I=");
    p = fmt_fixed(p, meas.i100, 2);
    p = app(p, " RELAY=");
    p = app(p, relay_on ? "ON" : "OFF");
    p = app(p, " FAULT=");
    p = app(p, FAULT_NAME[cur_fault]);
    p = app(p, " STATE=");
    p = app(p, state == ST_NORMAL ? "NORMAL" : (state == ST_LOCKOUT ? "LOCKOUT" : "TRIPPED"));
    app(p, "\r\n");
    uart_puts(b);
}

static void cmd_get(void)
{
    char b[96], *p = b;
    *p = 0;
    for (uint8_t i = 0; i < N_SET; i++)
    {
        p = app(p, SET_TBL[i].name);
        p = app(p, "=");
        p = fmt_fixed(p, cfg[i], SET_TBL[i].dec);
        p = app(p, i + 1 < N_SET ? " " : "\r\n");
    }
    uart_puts(b);
}

static void cmd_log(void)
{
    char b[40], *p;
    if (last_fault == F_NONE)
    {
        uart_puts("LAST=NONE\r\n");
        return;
    }
    p = app(b, "LAST=");
    p = app(p, FAULT_NAME[last_fault]);
    p = app(p, "@");
    if (last_fault == F_OC || last_fault == F_SHORT)
    {
        p = fmt_fixed(p, last_val, 2);
        p = app(p, "A");
    }
    else
    {
        p = fmt_fixed(p, last_val, 1);
        p = app(p, "V");
    }
    app(p, "\r\n");
    uart_puts(b);
}

static void process_line(char *line)
{
    char *cmd, *a1, *a2;
    char *p = line;

    for (char *q = line; *q; q++)
        if (*q >= 'a' && *q <= 'z')
            *q = (char)(*q - 32);

    while (*p == ' ')
        p++;
    cmd = p;
    while (*p && *p != ' ')
        p++;
    if (*p)
    {
        *p++ = 0;
        while (*p == ' ')
            p++;
    }
    a1 = p;
    while (*p && *p != ' ')
        p++;
    if (*p)
    {
        *p++ = 0;
        while (*p == ' ')
            p++;
    }
    a2 = p;

    if (!strcmp(cmd, "STATUS"))
        cmd_status();
    else if (!strcmp(cmd, "GET"))
        cmd_get();
    else if (!strcmp(cmd, "LOG"))
        cmd_log();
    else if (!strcmp(cmd, "SAVE"))
    {
        settings_save();
        uart_puts("OK SAVED\r\n");
    }
    else if (!strcmp(cmd, "RESET"))
    {
        clear_fault();
        uart_puts("OK\r\n");
    }
    else if (!strcmp(cmd, "DEFAULTS"))
    {
        settings_defaults();
        recompute_limits();
        uart_puts("OK (use SAVE to keep)\r\n");
    }
    else if (!strcmp(cmd, "HELP"))
        uart_puts("CMDS: STATUS GET SET SAVE RESET LOG DEFAULTS\r\n");
    else if (!strcmp(cmd, "SET"))
    {
        int8_t idx = find_setting(a1);
        uint16_t v;
        if (idx < 0 || !parse_num(a2, SET_TBL[idx].dec, &v))
        {
            uart_puts("ERR ARG\r\n");
        }
        else
        {
            uint16_t t[N_SET];
            memcpy(t, cfg, sizeof t);
            t[idx] = v;
            if (!validate_cfg(t))
            {
                uart_puts("ERR RANGE\r\n");
            }
            else
            {
                cfg[idx] = v;
                recompute_limits();
                uart_puts("OK\r\n");
            }
        }
    }
    else
        uart_puts("ERR CMD\r\n");
}

/* ---- hardware init ------------------------------------------------------ */
static void hw_init(void)
{
    LATD = 0x00; /* relay OFF before anything else          */
    TRISD = 0x00;
    TRISC = 0xC0; /* RC6/RC7 inputs for EUSART, others out   */
    LATC = 0x00;
    TRISA = 0xFF;
    TRISB = 0xFF;
    INTCON2bits.RBPU = 0; /* PORTB pull-ups on (buttons)             */

    CMCON = 0x07;  /* comparators off                         */
    ADCON1 = 0x0D; /* AN0, AN1 analog; rest digital; Vdd/Vss  */
    ADCON2 = 0xAA; /* right justified, 12 TAD, Fosc/32        */
    ADCON0 = 0x01;

    T1CON = 0x81; /* Timer1: 16-bit, Fosc/4, 1:1             */
    T0CON = 0x82; /* Timer0: 16-bit, 1:8 -> 1 ms tick        */
    TMR0H = 0xFD;
    TMR0L = 0x8F;

    TXSTA = 0x24; /* TXEN, BRGH                              */
    RCSTA = 0x90; /* SPEN, CREN                              */
    BAUDCON = 0x00;
    SPBRGH = 0;
    SPBRG = 129; /* 9600 baud @ 20 MHz, BRGH=1              */

    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    PIE1bits.RCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

/* ---- main --------------------------------------------------------------- */
void main(void)
{
    uint32_t last_disp = 0;
    char buf[RX_MAX];

    hw_init();
    settings_load();
    lcd_init();

    lcd_line(0, "V-PROTECTOR     ");
    lcd_line(1, defaults_used ? "DEFAULT SETTINGS" : "SETTINGS LOADED ");
    uart_puts(defaults_used ? "BOOT DEFAULTS\r\n" : "BOOT OK\r\n");
    for (uint8_t i = 0; i < 20; i++)
    {
        __delay_ms(50);
        CLRWDT();
    }

    for (;;)
    {
        CLRWDT();

        measure(&meas);
        protection_step(&meas);

        uint32_t now = millis();
        LED_OK = (state == ST_NORMAL);
        LED_FAULT = (cur_fault != F_NONE) || (state == ST_LOCKOUT);
        BUZZER = ((int32_t)(buzz_until - now) > 0);

        ui_handle(buttons_poll());

        if (line_ready)
        {
            uint8_t g = INTCONbits.GIE;
            INTCONbits.GIE = 0;
            strncpy(buf, (const char *)rx_buf, RX_MAX);
            buf[RX_MAX - 1] = 0;
            line_ready = false;
            INTCONbits.GIE = g;
            process_line(buf);
        }

        if ((now - last_disp) >= 200UL)
        {
            last_disp = now;
            if (menu_active)
                menu_draw();
            else
                display_update();
        }
    }
}