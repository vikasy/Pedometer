/*
  Author: Vikas Yadav (vikasy@gmail.com)
  Filename: pedometer.c
  Topic:  Accelerometer based Basic Step Detect and Count Algorithm

  This algorithm is based on analyzing forces acting on body while moving or sitting. 
  The forces can be measured using accelerometer (3-axis, AccX, AccY, and AccZ). 
  Among these three axis, the AccY is the most sensitive as it measures the ground 
  vertical reaction forces. The gyroscope data is not used in this algorithm for the 
  sensor frame data is very close to the ground fixed frame for the purpose of step 
  detection and also it consumes more power to have an algorithm based on gyroscope. 

  The raw sensor data AccY is preprocessed by passing through a digital 2nd order low 
  pass filter (cut-off at 3Hz). The filtered AccY is buffered in an input buffer and 
  the main algorithm is called only when the input buffer is full to reduce the algorithm 
  processing time. 

  During every run of the algorithm, the time derivative, DAccY, of the buffered filtered 
  AccY is computed using a digital 2nd order lead-lag filter (cut-off 4Hz). The basic idea 
  is to estimate real-time amplitude and frequency of the motion data and use it to 
  estimate step type and step counts. The amplitude and frequency of filtered AccY is 
  computed by locating its maximum, amax at tmax, and minimum, amin at tmin, values. 

  The instantaneous amplitude is estimated by taking difference between amax and amin. 
  A step is estimated by occurrence of two consecutive minimum values. The step count is 
  incremented for every occurrence of such consecutive pairs of minimum values. The 
  instantaneous time duration of one step is computed by taking difference between two 
  consecutive tmin time values (or tmax time values).

  The type of motion for a step e.g. walking, running, etc., is estimated based on 
  the estimated frequency and estimated amplitude of the step as shown in the table below:

  Type of Motion based on Estimated Frequency and Estimated Amplitude:
  SLOW_FREQ && SMALL_AMP => STATIONARY
  FAST_FREQ && LARGE_AMP => WALKING
  FAST_FREQ && LARGE_AMP => HOPPING
  FAST_FREQ && LARGE_AMP => RUNNING

  The algorithm is configured with the following values:
  SMALL_AMP = 5.0 m/s2
  LARGE_AMP = 15.0 m/s2 
  SLOW_FREQ = 0.5 Hz
  FAST_FREQ = 2.2 Hz

  The challenging part is to come up with the values of configurable parameters, which might 
  vary from user to user. The above values were obtained by performing Matlab analysis on 
  the provided sample user data. 
  
  Note: This program runs on sensor date read from an inptu file in lieu of real tiem sensor data.
  In presence of real tiem sensor data, this can be replaced by periodic sensor data read using 
  sensor manager apis to get sensor data.
  
  (c) 2017, Vikas Yadav

*/


#include "main.h"


/* Low pass filter for smoothening sensor input data */
static filter_t    lp_filter_x, lp_filter_y, lp_filter_z;
static filter_t    ll_filter_x, ll_filter_y, ll_filter_z;

/* Algo output data */
static algo_out_t  step_algo_output;

/* Sensor input data buffer for algo processing */
static float       AccBuff[NUM_DIM+1][SAMP_BUFF_LEN];

/* Number of steps during Walk, Run, and Hop motions */
static unsigned int num_steps_walk, num_steps_run, num_steps_hop;

/* Main entry point */
int main(int argc, char *argv[])
{
#define MAX_CHAR_PER_LINE     (120)

    FILE          *fpin, *fpout;
    unsigned int  rec_id = 0, sen_id = 0;
    char          date[12] = { 0 }, time[12] = { 0 };
    float         arx = 0.0f, ary = 0.0f, arz = 0.0f; /* m/s^2 */
    float         grx = 0.0f, gry = 0.0f, grz = 0.0f; /* rad/s */
    float         timestamp = 0.0f; /* sec */
    char          line_buff[MAX_CHAR_PER_LINE] = { 0 };
    unsigned int  skip_lines = 2;
    unsigned int  run_step_algo = 0;
    unsigned int  timeconst_samp = 0;
    char          *step_type;

    if( argc != 3) {
        printf("Usage: %s inputfile outputfile\n", argv[0]);
        exit(1);
    }

    fpin = fopen(argv[1], "r");
    if(fpin == NULL) {
        printf("Cannot open input file: %s\n", argv[1]);
        exit(1);
    }

    fpout = fopen(argv[2], "w");
    if(fpout == NULL) {
        printf("Cannot open output file: %s\n", argv[2]);
        exit(1);
    }

    /* Algorithm uses a 2nd order low pass filter to smoothen the input sensor data   */
    /* and a 2nd order lead-lag filter to estimate time derivate of input sensor data */
    
    /* Initialize 2nd order low pass fitler data, 3Hz cut-off */
    lp_filter_x.b0 = lp_filter_y.b0 = lp_filter_z.b0 = 7.2269463E-03f;
    lp_filter_x.b1 = lp_filter_y.b1 = lp_filter_z.b1 = 1.4453893E-02f;
    lp_filter_x.b2 = lp_filter_y.b2 = lp_filter_z.b2 = 7.2269463E-03f;
    lp_filter_x.a1 = lp_filter_y.a1 = lp_filter_z.a1 = -1.7455322E+0f;
    lp_filter_x.a2 = lp_filter_y.a2 = lp_filter_z.a2 = 7.7444003E-01f;
    lp_filter_x.prev_in = lp_filter_y.prev_in = lp_filter_z.prev_in = 0.0f;
    lp_filter_x.prev_prev_in = lp_filter_y.prev_prev_in = lp_filter_z.prev_prev_in = 0.0f;
    lp_filter_x.prev_out = lp_filter_y.prev_out = lp_filter_z.prev_out = 0.0f;
    lp_filter_x.prev_prev_out = lp_filter_y.prev_prev_out = lp_filter_z.prev_prev_out = 0.0f;
    timeconst_samp = (unsigned int)(SENSOR_SAMP_FREQ*0.075f) + 1;
    if (timeconst_samp > MAX_TC_SAMPLES)
        timeconst_samp = MAX_TC_SAMPLES;
    lp_filter_x.TC_samples = lp_filter_y.TC_samples = lp_filter_z.TC_samples = timeconst_samp;

    /* Initialize 2nd order lead lag fitler data, 4Hz cut-off */
    ll_filter_x.b0 = ll_filter_y.b0 = ll_filter_z.b0 = 2.5369363f;
    ll_filter_x.b1 = ll_filter_y.b1 = ll_filter_z.b1 = 0.0f;
    ll_filter_x.b2 = ll_filter_y.b2 = ll_filter_z.b2 = -2.5369363f;
    ll_filter_x.a1 = ll_filter_y.a1 = ll_filter_z.a1 = -1.6641912f;
    ll_filter_x.a2 = ll_filter_y.a2 = ll_filter_z.a2 = 0.71297842f;
    ll_filter_x.prev_in = ll_filter_y.prev_in = ll_filter_z.prev_in = 0.0f;
    ll_filter_x.prev_prev_in = ll_filter_y.prev_prev_in = ll_filter_z.prev_prev_in = 0.0f;
    ll_filter_x.prev_out = ll_filter_y.prev_out = ll_filter_z.prev_out = 0.0f;
    ll_filter_x.prev_prev_out = ll_filter_y.prev_prev_out = ll_filter_z.prev_prev_out = 0.0f;
    timeconst_samp = (unsigned int)(SENSOR_SAMP_FREQ*0.06f) + 1;
    if (timeconst_samp > MAX_TC_SAMPLES)
        timeconst_samp = MAX_TC_SAMPLES;
    ll_filter_x.TC_samples = ll_filter_y.TC_samples = ll_filter_z.TC_samples = timeconst_samp;

    /* Initialize algo output data struct */
    step_algo_output.prev_max = 0.0;
    step_algo_output.prev_max_ts = 0.0;
    step_algo_output.prev_min = 0.0;
    step_algo_output.prev_min_ts = 0.0;
    step_algo_output.step_count = 0;
    step_algo_output.step_type = 0;

    /* Skip the first two lines of input file */
    while( (skip_lines > 0) && (NULL != fgets(line_buff, MAX_CHAR_PER_LINE, fpin)) ) {
        skip_lines--;
    }
    if( skip_lines > 0 ) {
        printf("Cannot read first two lines of input file: %s\n", argv[2]);
        exit(1);
    }

    fprintf(fpout, "RECORD, TYPE, DATE, TIME, arx, ary, arz, grx, gry, grz, timestamp(sec), step_count, step_type, step_type_num\n");
    /* Process input sensor data from input file and save result in output file */
    while (NULL != fgets(line_buff, MAX_CHAR_PER_LINE, fpin))
    {
        sscanf(line_buff, "%d, %d, %[^,], %[^,], %f, %f, %f, %f, %f, %f\n",
            &rec_id, &sen_id, date, time, &arx, &ary, &arz, &grx, &gry, &grz);
        timestamp += SENSOR_SAMP_INTVL;

        run_step_algo = step_algo_preproc( timestamp, arx, ary, arz, grx, gry, grz );
        if (run_step_algo == 1) {
            /* Collected enough sensor data to run step detect and count */
            step_algo_run(&step_algo_output);
        }

        if( step_algo_output.step_type == STATIC )
            step_type = "STATIONARY";
        else if( step_algo_output.step_type == WALK )
            step_type = "WALKING";
        else if( step_algo_output.step_type == RUN )
            step_type = "RUNNING";
        else if( step_algo_output.step_type == HOP )
            step_type = "HOPPING";
         
        fprintf(fpout, "%d, %d, %s, %s, %f, %f, %f, %f, %f, %f, %f, %d, %s, %d\n", 
            rec_id, sen_id, date, time, arx, ary, arz, grx, gry, grz, 
            timestamp, step_algo_output.step_count, step_type, step_algo_output.step_type);
    }

    printf("Total motion duration is %f sec, which contains approximatly:\n %d Total number of steps including\n |---> %d steps of WALKING, \n |---> %d steps of RUNNING, and \n |---> %d steps of HOPPING.\n", timestamp, num_steps_walk+num_steps_run+num_steps_hop, num_steps_walk, num_steps_run, num_steps_hop);
    printf("Done.\n");
    exit(0);

}


/* Algo date preprocessing function called by Main to do some preprocessing of sensor input data 
*  and store the preprocessed data in a buffer.
*  Only call algorithm to run when the sensor input buffer is full, this saves power
*  Input: Timestamp in sec, AccX, AccY, AccZ, GyroX, GyroY, GyroZ data
*  Output: 1 if sensor input buffer is full, 0 otherwise
*/
static unsigned int step_algo_preproc(float timestamp, float arx, float ary, float arz, float grx, float gry, float grz)
{
    static unsigned int count = 0;
    unsigned int  ret_val = 0;
    float         ary_flt = 0.0f;

    /* Only use y-axis accelerometer data for algo      */
    /* Filter input sensor data before saving in buffer */
    ary_flt = apply_filter(&lp_filter_y, ary);
    AccBuff[CHY][count] = ary_flt;
    AccBuff[CHZ + 1][count] = timestamp;
    count = count + 1;
    if (count == SAMP_BUFF_LEN) {
        /* buffer is full, signal algo to run */
        ret_val = 1;
        count = 0;
    }

    return ret_val;

}


/* The main algorithm, processes the sensor input data in sensor input buffer 
*  stored by preproc function to detect step type and estimate step count.
*  The algorithm estimates frequency and amplitude of the accel y-axis data
*  by finding max and min values.
*  Max and min values have derivate of 0, thus a lead-lag filter is used to 
*  take derivative of the input data. The zero-crossings of derivative data
*  provides max and min values of input data.
*  The  zero crossing is computed by finding points where the data changes sign.
*  Input: Pointer to the algo output data
*  Output: None
*/
static void step_algo_run(algo_out_t *step_algo_output)
{
#define VERY_HIGH_VAL          ( 100000 )
#define NO_DETECT_DUR_SEC      ( 0.2f )    /* Duration to avoid very close peaks not related to step */
#define CLOSE_TO_ZERO          ( 1.5 )     /* Threshold to detect if peak is reasonable height       */
#define MAX_TIME_PERIOD_SEC    ( 1.5f )    /* Maximum time duration of one step                      */
#define SMALL_AMP              ( 5.0 )     /* Lower threshold for change in accel during a step      */
#define LARGE_AMP              ( 15.0 )    /* Higher threshold for change in accel during a step     */
#define SLOW_FREQ              ( 0.5 )     /* Lower threshold for step rate during a step            */
#define FAST_FREQ              ( 2.2 )     /* Lower threshold for step rate during a step            */

    /* Local Variables */
    unsigned int         i;
    float                prev_max_val, prev_max_ts, prev_min_val, prev_min_ts;
    float                new_max_val, new_max_ts, new_min_val, new_min_ts;
    float                avg_max_val = 0.0f, avg_min_val = 0.0f, avg_time_period = 0.0f;
    float                freq_est = 0.0f, amp_est = 0.0f;
    unsigned int         delta = 1;     /* distance between two samples to be compared   */
    unsigned int         count_max_det = 0, count_min_det = 0;
    unsigned int         TC_samples = 0;
    
    /* Local Static Variables to maintain state of algo and static array allocation  */
    static float         prev_amp_est = 0.0f, prev_freq_est = 0.0f;
    static unsigned int  amp_est_hold = 0, freq_est_hold = 0;
      static float         prevAccDer = 0.0f;
    static float         AccFilt[SAMP_BUFF_LEN + MAX_TC_SAMPLES] = { 0 };
    static float         TimeStamps[SAMP_BUFF_LEN + MAX_TC_SAMPLES] = { 0 };
    static float         AccDer[SAMP_BUFF_LEN] = { 0 }; /* Derivative of Accel Data */

    /* Initialize variables */
    new_max_val = -VERY_HIGH_VAL;
    new_min_val = VERY_HIGH_VAL;
    new_max_ts = 0.0f;
    new_min_ts = 0.0f;
    
    /* Prev max and min values and their timestamps are   */
    /* needed to estimate one step in cases where data for*/
    /* one step span accross multiple calls to this algo  */
    prev_max_val = step_algo_output->prev_max;
    prev_max_ts = step_algo_output->prev_max_ts;
    prev_min_val = step_algo_output->prev_min;
    prev_min_ts = step_algo_output->prev_min_ts;
    
    /* Delay between derivative and filtered data due to  */
    /* derivate (lead0lag) fiter delay                    */
    TC_samples = ll_filter_y.TC_samples;

    for (i = 0; i < SAMP_BUFF_LEN; i++) {
        /* Compute the derivative of filtered Y-axis Acc Data  */
        AccDer[i] = apply_filter(&ll_filter_y, AccBuff[CHY][i]);
        /* AccFilt and TimeStamps are extended buffers to save */
        /* prev TC_samples along with new Y-axis Acc Data      */
        AccFilt[TC_samples + i] = AccBuff[CHY][i];
        TimeStamps[TC_samples + i] = AccBuff[CHZ+1][i];
    }
    
    /* Find the new max/min values of Filtered Y-axis Acc Data and timestamp             */
    /* This is done by detecting rising/falling zero crossings in derivative of Acc Data */
    for (i = 0; i < SAMP_BUFF_LEN; i = i + delta) {
        if (i >= delta)
            prevAccDer = AccDer[i-delta];
        
        if (prev_max_ts <= prev_min_ts) {
            /* need to find the next max val(falling ZC) */
            if ((AccDer[i] < -EPSILON) && (prevAccDer >= 0.0f)) {
                /* Avoid searching very close to already found max value */
                if (TimeStamps[i] - prev_max_ts > NO_DETECT_DUR_SEC) {
                    new_max_val = AccFilt[i];
                    if (fabs(new_max_val) > CLOSE_TO_ZERO) {
                        new_max_ts = TimeStamps[i];
                        /* Clamp diff betweeen new_max_ts and prev_max_ts to avoid */
                        /* distortion in estimation of step frequency              */
                        if (new_max_ts > prev_max_ts + MAX_TIME_PERIOD_SEC)
                            prev_max_ts = new_max_ts - MAX_TIME_PERIOD_SEC;
                        /* Avoid max vlaue which is very close to prev min value   */
                        if (new_max_val - prev_min_val > CLOSE_TO_ZERO) {
                            avg_time_period = avg_time_period + (new_max_ts - prev_max_ts);
                            avg_max_val = avg_max_val + new_max_val;
                            count_max_det = count_max_det + 1;
                            prev_max_ts = new_max_ts;
                            prev_max_val = new_max_val;
                        }
                    }
                }
            }
        }
        else {
            /* need to find the next min val(rising ZC) */
            if ((AccDer[i] > EPSILON) && (prevAccDer <= 0.0f)) {
                /* Avoid searching very close to already found min value */
                if (TimeStamps[i] - prev_min_ts > NO_DETECT_DUR_SEC) {
                    new_min_val = AccFilt[i];
                    new_min_ts = TimeStamps[i];
                    /* Clamp diff betweeen new_max_ts and prev_max_ts to avoid */
                    /* distortion in estimation of step frequency              */
                    if (new_min_ts > prev_min_ts + MAX_TIME_PERIOD_SEC)
                        prev_min_ts = new_min_ts - MAX_TIME_PERIOD_SEC;
                    /* Avoid min vlaue which is very close to prev max value   */
                    if (prev_max_val - new_min_val > CLOSE_TO_ZERO) {
                        avg_time_period = avg_time_period + (new_min_ts - prev_min_ts);
                        avg_min_val = avg_min_val + new_min_val;
                        amp_est += prev_max_val - new_min_val;
                        /* A pair of max and min is one step and is equal to   */
                        /* the value of count_min_det                          */
                        count_min_det = count_min_det + 1;
                        prev_min_ts = new_min_ts;
                        prev_min_val = new_min_val;
                    }
                }
            }
        }
    }

    if (count_min_det > 0) {
        amp_est = amp_est / count_min_det;
        amp_est_hold = 0;
    }
    else {
        amp_est = prev_amp_est;
        amp_est_hold++;
    }
    if( amp_est_hold > BUFF_FACTOR ) {
        amp_est_hold = 0;
        amp_est = 0.0f;
    }

    if (count_max_det + count_min_det > 0) {
        avg_time_period = avg_time_period / (count_max_det + count_min_det);
    }
    if (avg_time_period > EPSILON) {
        freq_est = 1 / avg_time_period;
        freq_est_hold = 0;
    }
    else {
        freq_est = prev_freq_est;
        freq_est_hold++;
    }
    if( freq_est_hold > BUFF_FACTOR ) {
        freq_est_hold = 0;
        freq_est = 0.0f;
    }

    /* Save the last TC_samples for next run */
    for (i = 0; i < TC_samples; i++) {
        AccFilt[i] = AccBuff[CHY][SAMP_BUFF_LEN - TC_samples + i];
        TimeStamps[i] = AccBuff[CHZ+1][SAMP_BUFF_LEN - TC_samples + i];
    }
    /* Save selected algo data for the next run */
    prevAccDer = AccDer[SAMP_BUFF_LEN-1];
    prev_amp_est = amp_est;
    prev_freq_est = freq_est;

    /* Update the algo output */
    step_algo_output->prev_max = prev_max_val;
    step_algo_output->prev_max_ts = prev_max_ts;
    step_algo_output->prev_min = prev_min_val;
    step_algo_output->prev_min_ts = prev_min_ts;
    step_algo_output->step_count += count_min_det;
    if (amp_est <= SMALL_AMP) {
        if (freq_est <= SLOW_FREQ)
            /* STATIONARY */
            step_algo_output->step_type = STATIC;
        else {
            /* WALKING */
            step_algo_output->step_type = WALK;
            num_steps_walk += count_min_det;
        }
    }
    else if (amp_est >= LARGE_AMP) {
        if (freq_est >= FAST_FREQ) {
            /* RUNNING */
            step_algo_output->step_type = RUN;
            num_steps_run += count_min_det;
        }
        else {
            /* HOPPINNG */
            step_algo_output->step_type = HOP;
            num_steps_hop += count_min_det;
        }
    }
    else {
        if (freq_est >= FAST_FREQ) {
            /* RUNNING */
            step_algo_output->step_type = RUN;
            num_steps_run += count_min_det;
        }
        else {
            /* WALKING */
            step_algo_output->step_type = WALK;
            num_steps_walk += count_min_det;
        }
    }

}
 
/* Apply second order filter on input data 
*  Update filter state for the next run
*  Input: Pointer to filter state var, Input data to filter
*  Output: Filtered data
*/
static float apply_filter(filter_t *filt_data, float in_data)
{
    float out_data;

    /* compute new output */
    out_data = (filt_data->b0 * in_data) + (filt_data->b1 * filt_data->prev_in) + (filt_data->b2 * filt_data->prev_prev_in) \
        - (filt_data->a1 * filt_data->prev_out) - (filt_data->a2 * filt_data->prev_prev_out);
    
    /* update state of filter */
    filt_data->prev_prev_in = filt_data->prev_in;
    filt_data->prev_in = in_data;
    filt_data->prev_prev_out = filt_data->prev_out;
    filt_data->prev_out = out_data;

    return out_data;

}

