#include "driverlib.h"
#include "msp430.h"

#define one 0x60
#define two 0xDB 
#define three 0xF3
#define four 0x67
#define five 0xB7
#define six 0xBF
#define seven 0XE4
#define eight 0xFF
#define nine 0xF7

#define TIMER_A_PERIOD  4000
#define HIGH_COUNT      2000

Timer_A_outputPWMParam param;

// basically the same as above definitions, but useful for int parsing.
int charLUT[10] = {0xFC, 0x60, 0xDB, 0xF3, 0x67, 0xB7, 0xBF, 0xE4, 0xFF, 0xF7};
float global_temp_moisture = 0.0;
float global_temp_moisture_tens = 0.0;
float global_temp_moisture_ones = 0.0;
float global_temp_moisture_decimal = 0.0;
int global_temp = 0;
int global_temp_tens = 0;
int global_temp_ones = 0;
int global_temp_decimal = 0;
bool moisture_metric = false;

int error = 0;
bool error_too_high = false;
bool error_too_low = false;

void gpio_init(void){
    // Set up all the GPIO pins :D
    // Pins run parallel to ADC pins (was pretty confusing until datasheet said so)
    
    // Measures Moisture 1:
    #define GPIO_PORT_ADC8          GPIO_PORT_P8
    #define GPIO_PIN_ADC1           GPIO_PIN1
    #define GPIO_FUNCTION_ADC9       GPIO_PRIMARY_MODULE_FUNCTION
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_ADC8,
            GPIO_PIN_ADC1,
            GPIO_FUNCTION_ADC9);
    // Measures Moisture 2:
    #define GPIO_PORT_ADC1          GPIO_PORT_P1
    #define GPIO_PIN_ADC4           GPIO_PIN4
    #define GPIO_FUNCTION_ADC4       GPIO_PRIMARY_MODULE_FUNCTION
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_ADC1,
            GPIO_PIN_ADC4,
            GPIO_FUNCTION_ADC4);
    // Measures Moisture 3:
    #define GPIO_PORT_ADC1          GPIO_PORT_P1
    #define GPIO_PIN_ADC6           GPIO_PIN6
    #define GPIO_FUNCTION_ADC6       GPIO_PRIMARY_MODULE_FUNCTION
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_ADC1,
            GPIO_PIN_ADC6,
            GPIO_FUNCTION_ADC6);
    // Measures Temperature 1:
    #define GPIO_PORT_ADC1          GPIO_PORT_P1
    #define GPIO_PIN_ADC0           GPIO_PIN0
    #define GPIO_FUNCTION_ADC0       GPIO_PRIMARY_MODULE_FUNCTION
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_ADC1,
            GPIO_PIN_ADC0,
            GPIO_FUNCTION_ADC0);
    // Measures Temperature 2:
    #define GPIO_PORT_ADC1          GPIO_PORT_P1
    #define GPIO_PIN_ADC5           GPIO_PIN5
    #define GPIO_FUNCTION_ADC5       GPIO_PRIMARY_MODULE_FUNCTION
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_ADC1,
            GPIO_PIN_ADC5,
            GPIO_FUNCTION_ADC5);
    // Measures Temperature 3:
    #define GPIO_PORT_ADC1          GPIO_PORT_P1
    #define GPIO_PIN_ADC3           GPIO_PIN3
    #define GPIO_FUNCTION_ADC3       GPIO_PRIMARY_MODULE_FUNCTION
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_ADC1,
            GPIO_PIN_ADC3,
            GPIO_FUNCTION_ADC3);

    //SET GPIO AS OUTPUT PIN
    //Moisture 1 2.7
    GPIO_setAsOutputPin(
        GPIO_PORT_P2,
        GPIO_PIN7
        );
    //Moisture 2 8.2
    GPIO_setAsOutputPin(
        GPIO_PORT_P8,
        GPIO_PIN2
    );
    //Moisture 3 5.1
    GPIO_setAsOutputPin(
        GPIO_PORT_P5,
        GPIO_PIN1
        );
    //Temperature 1 8.0
    GPIO_setAsOutputPin(
        GPIO_PORT_P8,
        GPIO_PIN0
        );
    //Temperature 2 8.3
    GPIO_setAsOutputPin(
        GPIO_PORT_P8,
        GPIO_PIN3
        );
    //Temperature 3 2.5 
    GPIO_setAsOutputPin(
        GPIO_PORT_P2,
        GPIO_PIN5
        );

    //Buzzer 1.7
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN7
        );
    //Motor 5.3
    GPIO_setAsOutputPin(
        GPIO_PORT_P5,
        GPIO_PIN3
        );
    
    
    //Ensure all LED is off
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
}

// generic ADC initialization
void temp_init(void){
    ADC_init(ADC_BASE, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC, ADC_CLOCKDIVIDER_1);
    ADC_enable(ADC_BASE);
    ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_16_CYCLES, ADC_MULTIPLESAMPLESDISABLE);

    ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
}


// give the right display
void parse_global_temp(){
    global_temp_tens = global_temp/100;
    global_temp_ones = (global_temp/10)%10;
    global_temp_decimal = global_temp%10;
}

// param: inputSourceSelect -> one of the ADC pins specified in adc.h
void call_temp(uint8_t inputSourceSelect){
    temp_init();
    ADC_configureMemory(ADC_BASE, inputSourceSelect, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);
    //Delay between conversions
    __delay_cycles(5000);

    //Enable and Start the conversion
    //in Single-Channel, Single Conversion Mode
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
    // ADC_startConversion(ADC_BASE, ADC_SEQOFCHANNELS);
            
    //??? memory buffer saves only one inputsourceselect!!! Need to switch...
    // ADC_disableConversions(ADC_BASE, ADC_COMPLETECONVERSION);
    // while(ADC_isBusy(ADC_BASE)){
    //     __delay_cycles(1000);
    // }
    // ADC_stopConversions();

    //LPM0, ADC10_ISR will force exit
    __bis_SR_register(CPUOFF + GIE);
    //For debug only
    __no_operation();
}

//ADC10 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC_VECTOR)))
#endif
void ADC_ISR (void)
{
    switch (__even_in_range(ADCIV,12)){
        case  0: break; //No interrupt
        case  2: break; //conversion result overflow
        case  4: break; //conversion time overflow
        case  6: break; //ADC10HI
        case  8: break; //ADC10LO
        case 10: break; //ADC10IN
        case 12:        //ADC10IFG0
            //(Automatically clears ADC10IFG0 by reading memory buffer)
            if(moisture_metric){
                global_temp_moisture = (ADC_getResults(ADC_BASE)*10.0/1023.0)*100.0;
                global_temp = (int)global_temp_moisture;
                parse_global_temp();
            }
            else{
                global_temp = ADC_getResults(ADC_BASE);
                parse_global_temp();
            }

          //Clear CPUOFF bit from 0(SR)
            //Breakpoint here and watch ADC_Result
          __bic_SR_register_on_exit(CPUOFF);
          break;
        default: break;
    }
}

void clear_memory(void)
{
    LCD_E_clearAllMemory(LCD_E_BASE);
}

void configureCOMSEG (void)
{
    // L0, L1, L2, L3: COM pins
    // L0 = COM0, L1 = COM1, L2 = COM2, L3 = COM3
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_MEMORY_COM0);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_1, LCD_E_MEMORY_COM1);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_2, LCD_E_MEMORY_COM2);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_3, LCD_E_MEMORY_COM3);

}

void hold(int n){
    //Hold value
    int delay;
    for (delay = 0; delay <n; delay++)
        __delay_cycles(1000); 
}

void lcd_reset(void){
    //Turn on LCD
    LCD_E_on(LCD_E_BASE);
    hold(2000);
    //Turn off LCD
    LCD_E_off (LCD_E_BASE);
    
    // Clear LCD memory
    clear_memory();
    // Configure COMs and SEGs
    configureCOMSEG();
}

void lcd_config_display_mois(uint8_t mask1, uint8_t mask2, uint8_t mask3, uint8_t mask4, uint8_t mask5, uint8_t mask6,
                                uint8_t mask7, uint8_t mask8, uint8_t mask9, uint8_t mask10){
    //Temperature 2
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_4, mask1);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_5, mask2);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_6, mask3);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_7, mask4);
    
    //Example Temperature 2
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_8, mask5);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, mask6);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_11, mask7);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, mask8);
    
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_18, mask9);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_19, mask10);
}

void lcd_config_display_temp(uint8_t mask1, uint8_t mask2, uint8_t mask3, uint8_t mask4, uint8_t mask5, uint8_t mask6,
                                uint8_t mask7, uint8_t mask8, uint8_t mask9, uint8_t mask10){
    //Temperature 2
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_4, mask1);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_5, mask2);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_6, mask3);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_7, mask4);
    
    //Example Temperature 2
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_8, mask5);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, mask6);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_11, mask7);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, mask8);
    
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_3, mask9);
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_18, mask10);
}

int parse_int_to_hex(int n){
    return charLUT[n];
}

void check_for_errors(){
    if(moisture_metric){
        if((global_temp_moisture*10.0/1023.0*100.0)/10 > 100){
            error = 1;
            error_too_high = true;
            return;
        }
        else if((global_temp_moisture*10.0/1023.0*100.0)/10 < 70){
            error = 1;
            error_too_low = true;
            return;
        }

    }
    else{
        if(global_temp/10 > 30){
            error = 1;
            error_too_high = true;
            return;
        }
        else if(global_temp/10 < 22){
            error = 1;
            error_too_low = true;
            return;
        }
    }
}

void Init_Buzzer(void)
{
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //1000
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //500


    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
}

void diagnostics(){
    
    //Make sure all LED are off
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7); //buzzer
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); //motor

     // Clear LCD memory
    clear_memory();
    // Configure COMs and SEGs
    configureCOMSEG();
    
    //Write Error on LCD
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_4, 0x9F); //Displays E
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_6, 0x0B); //Displays r
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_8, 0x0B); //Displaya r
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, 0x3B);//Displays o
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, 0x0B); //Displays r
    LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_12, 0x01);//Displays !
    
    LCD_E_on(LCD_E_BASE);
    
    //Turn Buzzer on
    Init_Buzzer();
    Timer_A_outputPWM(TIMER_A0_BASE, &param); //Turn on PWM
        
    while (error == 1){
     GPIO_toggleOutputOnPin(GPIO_PORT_P5, GPIO_PIN3);

        //All LED is off
        bool led_status = false;
        
        //Turn on Motor for Testing to show it works
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); 
        
        //Check Temperature 1
        moisture_metric = false;
        call_temp(ADC_INPUT_A0);
        //Call error function
        check_for_errors();
        if (error_too_high){
            //Turn on LED
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
            led_status = true;
        }
        else if (error_too_low){
            //Blink LED
            GPIO_toggleOutputOnPin(GPIO_PORT_P8, GPIO_PIN0);
            led_status = true;
        }
        else {
            //Turn off LED
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
        }
        error_too_high = false;
        error_too_low = false;
        
        //Check Moisture 1
        moisture_metric = true;
        call_temp(ADC_INPUT_A9);
        //Call error function
        check_for_errors();
        if (error_too_high){
            //Turn on LED
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
            led_status = true;
        }
        else if (error_too_low){
            //Blink LED
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN7);
            led_status = true;
        }
        else {
            //Turn off LED
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
        }
        error_too_high = false;
        error_too_low = false;

        //Check Temperature 2
        moisture_metric = false;
        call_temp(ADC_INPUT_A5);
        //Call error function
        check_for_errors();
        if (error_too_high){
            //Turn on LED
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
            led_status = true;
        }
        else if (error_too_low){
            //Blink LED
            GPIO_toggleOutputOnPin(GPIO_PORT_P8, GPIO_PIN3);
            led_status = true;
        }
        else {
            //Turn off LED
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
        }
        error_too_high = false;
        error_too_low = false;
        
        //Check Moisture 2
        moisture_metric = true;
        call_temp(ADC_INPUT_A4);
        //Call error function
        check_for_errors();
        if (error_too_high){
            //Turn on LED
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
            led_status = true;
        }
        else if (error_too_low){
            //Blink LED
            GPIO_toggleOutputOnPin(GPIO_PORT_P8, GPIO_PIN2);
            led_status = true;
        }
        else {
            //Turn off LED
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        }
        error_too_high = false;
        error_too_low = false;

        //Check Temperature 3
        moisture_metric = false;
        call_temp(ADC_INPUT_A3);
        //Call error function
        check_for_errors();
        if (error_too_high){
            //Turn on LED
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
            led_status = true;
        }
        else if (error_too_low){
            //Blink LED
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN5);
            led_status = true;
        }
        else {
            //Turn off LED
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        }
        error_too_high = false;
        error_too_low = false;
        
        //Check Moisture 3
        moisture_metric = true;
        call_temp(ADC_INPUT_A6);
        //Call error function
        check_for_errors();
        if (error_too_high){
            //Turn on LED
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
            led_status = true;
        }
        else if (error_too_low){
            //Blink LED
            GPIO_toggleOutputOnPin(GPIO_PORT_P5, GPIO_PIN1);
            led_status = true;
        }
        else {
            //Turn off LED
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
        }
        error_too_high = false;
        error_too_low = false;


        // Call another function that blinks or constant light
        //
        //If led thing is still 0 then error ==0
        if(!led_status){
            error = 0;
        }
        hold(250);
    }
    //Clear the LCD
    lcd_reset();
    //Ensure all the GPIO are OFF
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); //Turn off motor
    Timer_A_stop(TIMER_A0_BASE); //Turns off buzzer
    
    return;
}

void main(void)
{
	// Hold watchdog
    WDT_A_hold(WDT_A_BASE);

    //Port select XT1
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P4,
        GPIO_PIN1 + GPIO_PIN2,
        GPIO_PRIMARY_MODULE_FUNCTION
        );

    //Set external frequency for XT1
    CS_setExternalClockSource(32768);

    //Select XT1 as the clock source for ACLK with no frequency divider
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);

    //Start XT1 with no time out
    CS_turnOnXT1(CS_XT1_DRIVE_0);

	//clear all OSC fault flag
	CS_clearAllOscFlagsWithTimeout(1000);

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    // L0~L26 & L36~L39 pins selected
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_SEGMENT_LINE_26);
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_36, LCD_E_SEGMENT_LINE_39);

    LCD_E_initParam initParams = LCD_E_INIT_PARAM;
    initParams.clockDivider = LCD_E_CLOCKDIVIDER_8;
    initParams.muxRate = LCD_E_4_MUX;
    initParams.segments = LCD_E_SEGMENTS_ENABLED;

    // Init LCD as 4-mux mode
    LCD_E_init(LCD_E_BASE, &initParams);

    // LCD Operation - Mode 3, internal 3.08v, charge pump 256Hz
    LCD_E_setVLCDSource(LCD_E_BASE, LCD_E_INTERNAL_REFERENCE_VOLTAGE, LCD_E_EXTERNAL_SUPPLY_VOLTAGE);
    LCD_E_setVLCDVoltage(LCD_E_BASE, LCD_E_REFERENCE_VOLTAGE_3_08V);

    LCD_E_enableChargePump(LCD_E_BASE);
    LCD_E_setChargePumpFreq(LCD_E_BASE, LCD_E_CHARGEPUMP_FREQ_16);
    
    // Actual logic here:
    
    // Let's enable internal sensors for testing purposes without breadboard
    PMM_enableInternalReference();
    while (PMM_REFGEN_NOTREADY == PMM_getVariableReferenceVoltageStatus());
    
    // Initialize our necessary pins and modules!
    gpio_init();
    // temp_init();
    
    while (error == 0){
        lcd_reset();
        
        // Temperature display
        // Temperature 1:
        moisture_metric = false;
        call_temp(ADC_INPUT_A0);
        lcd_config_display_temp(0x80, 0x50, one, 0x04, parse_int_to_hex(global_temp_tens), parse_int_to_hex(global_temp_ones),
                            0x01, parse_int_to_hex(global_temp_decimal), 0x04, 0x9C);
        check_for_errors();
        if(error == 1){
            diagnostics();
            call_temp(ADC_INPUT_A0);
        }

        lcd_reset();
        
        // Moisture display (check if delay is enough for propogation)
        // Moisture 1:
        moisture_metric = true;
        call_temp(ADC_INPUT_A9);
        lcd_config_display_mois(0x6C, 0xA0, one, 0x04, parse_int_to_hex(global_temp_tens), parse_int_to_hex(global_temp_ones),
                    0x01, parse_int_to_hex(global_temp_decimal), 0x27, 0xAA);
        check_for_errors();
        if(error == 1){
            diagnostics();
            call_temp(ADC_INPUT_A9);
        }

        lcd_reset();

        // Temperature 2:
        moisture_metric = false;
        call_temp(ADC_INPUT_A3);
        lcd_config_display_temp(0x80, 0x50, two, 0x04, parse_int_to_hex(global_temp_tens), parse_int_to_hex(global_temp_ones),
                            0x01, parse_int_to_hex(global_temp_decimal), 0x04, 0x9C);
        check_for_errors();
         if(error == 1){
             diagnostics();
             call_temp(ADC_INPUT_A3);
        }
        
        lcd_reset();
        
        // Moisture 2:
        moisture_metric = true;
        call_temp(ADC_INPUT_A4);
        lcd_config_display_mois(0x6C, 0xA0, two, 0x04, parse_int_to_hex(global_temp_tens), parse_int_to_hex(global_temp_ones),
                            0x01, parse_int_to_hex(global_temp_decimal), 0x27, 0xAA);
        check_for_errors();
        if(error == 1){
            diagnostics();
            call_temp(ADC_INPUT_A4);
        }
        
        lcd_reset();

        // Temperature 3:
        moisture_metric = false;
        call_temp(ADC_INPUT_A5);
        lcd_config_display_temp(0x80, 0x50, three, 0x04, parse_int_to_hex(global_temp_tens), parse_int_to_hex(global_temp_ones),
                            0x01, parse_int_to_hex(global_temp_decimal), 0x04, 0x9C);
        check_for_errors();
        if(error == 1){
            diagnostics();
            call_temp(ADC_INPUT_A5);
        }
        
        lcd_reset();
        
        // Moisture 3:
        moisture_metric = true;
        call_temp(ADC_INPUT_A6);
        lcd_config_display_mois(0x6C, 0xA0, three, 0x04, parse_int_to_hex(global_temp_tens), parse_int_to_hex(global_temp_ones),
                            0x01, parse_int_to_hex(global_temp_decimal), 0x27, 0xAA);
        check_for_errors();
        if(error == 1){
            diagnostics();
            call_temp(ADC_INPUT_A6);
        }
        
        lcd_reset();

        
    }

    // Enter LPM3.5
    PMM_turnOffRegulator();
    __bis_SR_register(LPM4_bits | GIE);
}


