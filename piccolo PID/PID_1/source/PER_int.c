/****************************************************************
 * FILENAME:     PER_int.c
 * DESCRIPTION:  periodic interrupt code
 * AUTHOR:       Mitja Nemec
 * DATE:         16.1.2009
 *
 ****************************************************************/
#include    "PER_int.h"
#include    "TIC_toc.h"

// za izracunun napetostni
long napetost_raw = 0;
long napetost_offset = 0;
// the max voltage devided by 12bits of ad converter
float napetost_gain = 3.3/4096;
float napetost = 0.0;

// za izracun toka
long tok_raw = 0;
long tok_offset = 2048;
float tok_gain = (1.0/5.1)*(1.0/50.0)*(3.3/4096);
float tok = 0.0;

// vklopno razmerje
float duty = 0.0;

// generiranje željene vrednosti
float ref_counter = 0;
float ref_counter_prd = SWITCH_FREQ;
float ref_counter_cmpr = 500;

float ref_value = 0;
float ref_value_high = 2.5;
float ref_value_low = 0.5;

// za kalibracijo preostale napetosti tokovne sonde
bool    offset_calib = TRUE;
long     offset_counter = 0;
float   offset_filter = 0.005;

// za oceno obremenjenosti CPU-ja
float   cpu_load  = 0.0;
long    interrupt_cycles = 0;

/**************************************************************
 * spremenljivke, ki jih potrebujemo za regulator
 **************************************************************/
float zeljena = 2.0;

/***********************/
float desired_value = 2.0;
double Kp = 0.4;
double Ki = 0.0015;
double Kd = 0;
double Ka = 0;
double Kb = 0;
float error = 0;
double prev_error = 0;
float dt = 1; //40000;
float epsilon = 0.01;
float max_voltage = 3.3;
float min_voltage = 1;
float integral = 0;
float prev_integral = 0;
float derivative = 0;
float duty_max = 1;
float duty_min = 0.1;
double output_PID = 0;
double output_PID_1 = 0;
float duty_1 = 0;
/***********************/





// spremenljikva s katero štejemo kolikokrat se je prekinitev predolgo izvajala
int interrupt_overflow_counter = 0;


/**************************************************************
 * Prekinitev, ki v kateri se izvaja regulacija
 **************************************************************/
#pragma CODE_SECTION(PER_int, "ramfuncs");
void interrupt PER_int(void)
{
	/* lokalne spremenljivke */

	// najprej povem da sem se odzzval na prekinitev
	// Spustimo INT zastavico casovnika ePWM1
	EPwm1Regs.ETCLR.bit.INT = 1;
	// Spustimo INT zastavico v PIE enoti
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

	// pozenem stoprico
	interrupt_cycles = TIC_time;
	TIC_start();

	// izracunam obremenjenost procesorja
	cpu_load = (float)interrupt_cycles / (CPU_FREQ/SWITCH_FREQ);

	// pocakam da ADC konca s pretvorbo
	ADC_wait();

	// ali kalibriram preostalo napetost tokovne sonde
	// for getting the proper offset value of the current
	// it's not completely neccesary, but it's better to get
	// the proper value of the offset which differ for every uC
	if (offset_calib == TRUE)
	{
		tok_raw = TOK;

		tok_offset = (1.0 - offset_filter) * tok_offset + offset_filter * tok_raw;

		offset_counter = offset_counter + 1;
		if ( offset_counter == SWITCH_FREQ)
		{
			offset_calib = FALSE;
		}

	}
	// sicer pa normalno obratujem
	else
	{
		// preracunam napetost
		napetost_raw = NAPETOST_ZA;
		napetost = napetost_raw * napetost_gain;

		tok_raw = TOK - tok_offset;
		tok = -tok_raw * tok_gain;

		// generiram zeljeno vrednost
		ref_counter = ref_counter + 1;
		if (ref_counter >= ref_counter_prd)
		{
			ref_counter = 0;
		}

		// stopnicasta zeljena vrednost
		if (ref_counter > ref_counter_cmpr)
		{
			ref_value = ref_value_low;
		}
		else
		{
			ref_value = ref_value_high;
		}


		desired_value = ref_value;
		/*******************************************************
		 * Tukaj pride koda regulatorja
		 *******************************************************/

		/*************************************************/
		error = desired_value - napetost;

		/*
// Parallel PID controler
       // if(abs(error) > epsilon)
      //  {
        	integral = integral + error * dt;
        	//derivative = (error - prev_error)/dt;
        	derivative = (-1) * (napetost) / dt;
    //    }

        output_PID = (Kp * error + Ki * integral + Kd * derivative);
        duty = (output_PID + desired_value) / max_voltage;

        // In  the case of overshots, the Ki is set to 0
        output_PID_1 = (Kp * error + 0 * integral + Kd * derivative);
        duty_1 = (output_PID_1 + desired_value) / max_voltage;

        if(duty > duty_max)
        {
        // Duty_max should be changed into duty_1 in the case of overshots
        	duty = duty_1;
        }
        else if(duty < duty_min)
        {
        	duty = duty_min;
        }

        prev_error = error;
		 */



		// Series PID controler

		Ka = Kp;
		Kb = Ki/Kp;

		integral = prev_integral + Kb *error*dt;

		output_PID = (Ka * error +  integral);
		// duty = (output_PID);
		duty = (output_PID + desired_value) / max_voltage;
		/*
        // In  the case of overshots, the Ki is set to 0
        output_PID_1 = (Ka * error + 0 * integral);
        duty_1 = (output_PID_1 + 0*desired_value) / max_voltage;*/

		if(duty > duty_max)
		{
			integral=prev_integral;
			output_PID = (Ka * error +  integral);
			duty = (output_PID + desired_value) / max_voltage;
			// Duty_max should be changed into duty_1 in the case of overshots
			//duty = duty_1;
		}

		else if(duty < duty_min)
		{
			integral=prev_integral;
			output_PID = (Ka * error +  integral);
			duty = (output_PID + desired_value) / max_voltage;
			//duty = duty_min;
		}


		prev_integral=integral;

		//duty = ref_value;

		// osvežim vklono razmerje
		PWM_update(duty);

		// spavim vrednosti v buffer za prikaz
		// to put the values into the buffer for displying them
		DLOG_GEN_update();
	}


	/* preverim, èe me sluèajno èaka nova prekinitev.
       èe je temu tako, potem je nekaj hudo narobe
       saj je èas izvajanja prekinitve predolg
       vse skupaj se mora zgoditi najmanj 10krat,
       da reèemo da je to res problem
	 */
	if (EPwm1Regs.ETFLG.bit.INT == TRUE)
	{
		// povecam stevec, ki steje take dogodke
		interrupt_overflow_counter = interrupt_overflow_counter + 1;

		// in ce se je vse skupaj zgodilo 10 krat se ustavim
		// v kolikor uC krmili kakšen resen HW, potem moèno
		// proporoèam lepše "hendlanje" takega dogodka
		// beri:ugasni moènostno stopnjo, ...
		if (interrupt_overflow_counter >= 10)
		{
			asm(" ESTOP0");
		}
	}

	// stopam
	TIC_stop();

}   // end of PWM_int

/**************************************************************
 * Funckija, ki pripravi vse potrebno za izvajanje
 * prekinitvene rutine
 **************************************************************/
void PER_int_setup(void)
{

	// Proženje prekinitve
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    //sproži prekinitev na periodo
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;         //ob vsakem prvem dogodku
	EPwm1Regs.ETCLR.bit.INT = 1;                //clear possible flag
	EPwm1Regs.ETSEL.bit.INTEN = 1;              //enable interrupt

	// inicializiram data logger
	dlog.trig_value = SWITCH_FREQ - 10;    // specify trigger value
	dlog.slope = Positive;                 // trigger on positive slope
	dlog.prescalar = 1;                    // store every  sample
	dlog.mode = Normal;                    // Normal trigger mode
	dlog.auto_time = 100;                  // number of calls to update function
	dlog.holdoff_time = 100;               // number of calls to update function

	dlog.trig = &ref_counter;
	dlog.iptr1 = &napetost;
	dlog.iptr2 = &tok;


	// registriram prekinitveno rutino
	EALLOW;
	PieVectTable.EPWM1_INT = &PER_int;
	EDIS;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
	IER |= M_INT3;
	// da mi prekinitev teèe  tudi v real time naèinu
	// (za razhoršèevanje main zanke in BACK_loop zanke)
	SetDBGIER(M_INT3);
}
