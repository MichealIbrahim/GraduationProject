#include <18F1330.h>

#fuses NOPROTECT,INTRC,NOMCLR,BROWNOUT         


                                                //   h1q1        h1q2       h3q3        h2q4
#use delay(internal=32MHz)                      //  (1,1)   ,   (1,0)   ,   (0,1)   ,   (0,0)
                                                //  0-5     6   6-11   12   12-15  16   16-23 0
                                               //  0 5     6   6      0     0  6  6     6   0
                                               
//--------------------------------------------------
#byte PeriodH = 0xF95
#byte PeriodL = 0xF96
#byte PTCON0 = 0xF9A

void set_period(int16 p);  //used to change period 
//--------------------------------------------------
// indices of 3 phases and flags to indicate halves, and they are initialized 120 degrees apart
// 120/360 * 24 = 8 so each phase are 8 values ahead of its previos
int8 p1 = 0;
int8 p2 = 8;   
int8 p3 = 4;// 16 = half + 4 

int1 half1 = 1;
int1 half2 = 1;
int1 half3 = 0;

//--------------------------------------------------
#define N 12  // number of levels in half   
//-------------------------------------------------
//start copy
const int16 state1[] = {0,824,1592,2251,2757,3076,3184,3076,2757,2251,1592,824};


#define period_End   850
#define period_Begin 2083
#define period_exact_end 833
#define End_PSC 2
#define Estate 1
#define i_end 3
#define Period_Threshold 700// period will always be between 700 and 1400 -> PWM frequency between 5714Hz and 11428Hz



//end copy
//---------------------------------------------------
//delay after changes ,due to interrupts this delay will increase slightly
#define change_delay_starting 75   //controls speeding up time
#define change_delay_stopping 100  //controls stopping time  

#define period_at_stop period_Begin 
//----------------------------------------------------

int1 do_action = 0 ;
int1 stop = 0 ;
unsigned int16 period ;
int8 post_sc ;
unsigned int16 new_avg ;
int8 new_state ;
int8 state = 4 ;
int8 c_int = 1 ;
int8 new_post_sc = 16;
int8 i = 0 ;
unsigned int16 avg =0;
unsigned int16 change = 0 ;
int32 temp;
//---------------------------------------------------
//--------------------------------------------------

#define Dead_Time 32                   //dead time used and its clock
#define DEAD_TIME_CLK PWM_CLOCK_DIV_4

#define st_button pin_a0              //LED and relay pins and trigger configuration
#define ispressed TRUE
#define reset_led pin_b2
#define ready_led pin_b3
#define led_on 0
#define led_off 1

#define Relay Pin_A3
#define close 1
#define open 0

#define debouncin_limit 125      //for the button

//---------------------------------------------------
//---------------------------------------------------

//----------------------------------------------------

#define current state1


#INT_PWMTB

void isr_PWMTB()
{  
   if(c_int != post_sc ){c_int++; return;}//software postscaler ()
   c_int = 1 ;
   if(do_action)   //changes only happens at end of pulses 
   {
      set_period(period); 
      avg = new_avg;
      state = new_state;
      post_sc = new_post_sc;

      do_action = 0 ;
   }
   if(stop)   //stopping protocole 
   {
      disable_interrupts(INT_PWMTB);
      set_power_pwm0_duty(0);
      set_power_pwm2_duty(0);
      set_power_pwm4_duty(0);
      stop = 0 ;    
      return;
   }
   if(half1)   set_power_pwm0_duty(avg+(current[p1]>>state));    //LOOK at 3phase Induction Motor Inverter code segment documentation to understand the calculations 
   else        set_power_pwm0_duty(avg-(current[p1]>>state));
   
   if(half2)   set_power_pwm2_duty(avg+(current[p2]>>state));
   else        set_power_pwm2_duty(avg-(current[p2]>>state));
   
   if(half3)   set_power_pwm4_duty(avg+(current[p3]>>state));
   else        set_power_pwm4_duty(avg-(current[p3]>>state));
   p1++;p2++;p3++;
   if(p1 == N) {p1 = 0 ; half1 = ~half1;}
   if(p2 == N) {p2 = 0 ; half2 = ~half2;}
   if(p3 == N) {p3 = 0 ; half3 = ~half3;}
}

void main()
{
   period = period_begin;  
   int16 counter=0;
   int1 state_f = 0;   //state = 0  stopped or stopping , state = 1 at max speed going for it 

  
   setup_adc(ADC_OFF);
   setup_adc_ports(NO_ANALOGS);
   set_tris_a(0b11111101);  
   set_tris_b(0b00000000);
   output_bit(relay,open);  //relay must be open first (high voltage not connected)
   
   enable_interrupts(GLOBAL);

   state = 4 ;   
   avg = (period_Begin)<<1;
   setup_power_pwm(PWM_CLOCK_DIV_4 | PWM_FREE_RUN ,1,0,period_at_stop-1,0,1,Dead_Time);
   post_sc = 16 ;                                        //start with a PS of 16 for smallest frequencies
   setup_power_pwm_pins(PWM_COMPLEMENTARY,PWM_COMPLEMENTARY,PWM_COMPLEMENTARY,PWM_COMPLEMENTARY); //setting up pwm pins 
   set_power_pwm0_duty(0);                               //lower mosfets are open when at stop
   set_power_pwm2_duty(0);
   set_power_pwm4_duty(0);
   
   output_bit(ready_led,led_off);
   output_bit(reset_led,led_on);  //one second for reset LED 
   delay_ms(1000); 
   output_bit(reset_led,led_off);

   delay_ms(200);             //extra time before closing relay
   output_bit(relay,close);   
   delay_ms(250);     //time to make sure everything is stable after connecting high voltage
   while(1)
   {
      //-------------------------------------------------------------------  button check
      
      output_bit(ready_led,led_on);  //signal that button can be pressed
      while(input(st_button) != ispressed);
      while(1)
      {
         if(input(st_button) == ispressed)   counter = 0 ;
         else counter++;
         
         if(counter >= debouncin_limit)
         {
            counter = 0 ;
            if(state_f == 1 ) state_f = 0 ;
            else
            {     
               i = 0 ;    
               new_state = 4 ;
               new_post_sc = 16;
               period = period_begin;
               new_avg = period<<1 ;
               do_action = 1;
               enable_interrupts(INT_PWMTB);
               while(do_action);
               
               delay_ms(change_delay_starting);
               
               state_f = 1 ;
            }
            break;
         }
      
      }
      output_bit(ready_led,led_off);   //ready led off means motor is starting or stopping
      //--------------------------------------------------------------------
      
     
      while(1)  //stopping and starting code 
      {  
         //---------------------------------------------------------------------
         // stops at proper period and i(post scaler) 
         if( period <= period_End && i == i_end && state_f )   //end protocols to insure variables are at their end state 
         {
            
            i = i_end;       
            period = period_exact_end;
            new_state = Estate ;      
            new_avg = period<<1 ;
            new_post_sc = END_PSC;
            do_action = 1; //wait for change to be executed 
            while(do_action);
   
            break; //max speed reached
         
         }
         else if(period >= period_begin && i == 0 && !state_f) 
         {
            stop = 1 ; //signal to stop
            while(stop);
            break; //speed stopped
         } 
         //------------------------------------------------------------------- 
         else // the i variable indicated which postscaler and the rest of the variables are calculated using it
         {

            //-------- calculating change for this cycle    -> 2.5 HZ change
            temp = (make32(6250)<<i) - period;
            change = _mul(period,period)/temp;                       
            

            
            if(state_f) period = period - change; // -change = frequency increasing
            else period = period + change;        //  +change ->frequency decreasing
            
            // if period is less than Threshold while going to max speed period gets multiplied by two and presecaller divided by 2 
            if((period < period_threshold) &&  i != 4 &&  state_f ){ i++ ; period = period<<1 ;}
            
            // if period is more than 2*Threshold while stopping period gets divided by two and presecaller multiplied by 2
            
            if((period > (period_threshold<<1)) &&  i != 0 && !state_f ){ i-- ; period = period>>1 ;}
            new_state = 4 - i ; // the amount of bits in shift right operation in the interrupt 

            new_avg = period<<1 ;
            new_post_sc = 1<<new_state ;  // 2^(4-i)
            do_action = 1 ;
            

            if(state_f) delay_ms(change_delay_starting);    //sometimes slowing down needs more time
            else delay_ms(change_delay_stopping);
            
         }
         

         while(do_action);
         
      }
      
   }
   
}




void set_period(int16 p)
{
   p -= 1 ;
   periodL = P;
   periodH = (p>>8);
}




