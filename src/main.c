#include <zephyr/kernel.h>        // Funções básicas do Zephyr (ex: k_msleep, k_thread, etc.)
#include <zephyr/device.h>        // API para obter e utilizar dispositivos do sistema
#include <zephyr/drivers/gpio.h>  // API para controle de pinos de entrada/saída (GPIO)
#include <pwm_z42.h>        // Biblioteca personalizada com funções de controle do TPM (Timer/PWM Module)
#include <zephyr/drivers/adc.h>


// resistor em conjunto com o LDR: 10kohms


#define ADC_RESOLUTION   12
#define ADC_GAIN   ADC_GAIN_1
#define ADC_REFERENCE   ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME   ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID_N   8  //PTB 0 --1
#define ADC_CHANNEL_ID_S   9 // PTB 1 --2
#define ADC_CHANNEL_ID_L   12// PTB 2 --3
#define ADC_CHANNEL_ID_O   13//PTB 3 --4
#define ADC_VREF_MV   3300
#define DELAY_LDR_MS   300 // a ser definido
#define TPM_MODULE 7500
#define margem_LDR 50 // a ser definido
#define Delay_Leitura 200 // a ser definido
#define Noite   3000 // a ser definido
#define Tempo_dormir  15000


static int16_t sample_buffer_N;
static int16_t sample_buffer_S;
static int16_t sample_buffer_L;
static int16_t sample_buffer_O;




//orientação: plano cartesiano: sentido antihorário- visto de cima do servomotor
uint16_t zero = TPM_MODULE*0.025; //0,5ms
uint16_t meio = TPM_MODULE*0.075; //1,5ms -> noventa graus
uint16_t max = TPM_MODULE*0.125; //2,5ms
uint16_t descanso =TPM_MODULE*0.058; //1ms-> quarenta e cinco graus




uint16_t movimento_t = TPM_MODULE*0.058; //sessenta graus
uint16_t movimento_b = TPM_MODULE*0.075; //noventa graus
uint16_t mexida= TPM_MODULE*0.0008;//mexer muito pouco: 0,72 graus




int main(void) {




    const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));




    if(!device_is_ready(adc_dev)) {
        printk("ADC não está pronto\n");
        return 1;
    }


    struct adc_channel_cfg channel1_cfg = {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = ADC_CHANNEL_ID_N,
        .differential = 0,
    };


    if (adc_channel_setup(adc_dev, &channel1_cfg) !=0) {
        printk("Erro ao configurar canal ADC\n");
        return 1;
    }
   
    struct adc_channel_cfg channel2_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID_S,
    .differential = 0,
    };


    if (adc_channel_setup(adc_dev, &channel2_cfg) !=0) {
        printk("Erro ao configurar canal ADC\n");
        return 1;
    }

        struct adc_channel_cfg channel3_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID_L,
    .differential = 0,
    };





    if (adc_channel_setup(adc_dev, &channel3_cfg) !=0) {
        printk("Erro ao configurar canal ADC\n");
        return 1;
    }




        struct adc_channel_cfg channel4_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID_O,
    .differential = 0,
    };


    if (adc_channel_setup(adc_dev, &channel4_cfg) !=0) {
        printk("Erro ao configurar canal ADC\n");
        return 1;
    }


    struct adc_sequence sequence1 = {
        .channels = BIT(ADC_CHANNEL_ID_N),
        .buffer = &sample_buffer_N,
        .buffer_size = sizeof(sample_buffer_N),
        .resolution = ADC_RESOLUTION,
    };








    struct adc_sequence sequence2 = {
        .channels = BIT(ADC_CHANNEL_ID_S),
        .buffer = &sample_buffer_S,
        .buffer_size = sizeof(sample_buffer_S),
        .resolution = ADC_RESOLUTION,
    };


    struct adc_sequence sequence3 = {
        .channels = BIT(ADC_CHANNEL_ID_L),
        .buffer = &sample_buffer_L,
        .buffer_size = sizeof(sample_buffer_L),
        .resolution = ADC_RESOLUTION,
    };




    struct adc_sequence sequence4 = {
        .channels = BIT(ADC_CHANNEL_ID_O),
        .buffer = &sample_buffer_O,
        .buffer_size = sizeof(sample_buffer_O),
        .resolution = ADC_RESOLUTION,
    };

    pwm_tpm_Init(TPM1, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);
    pwm_tpm_Init(TPM0, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);



    pwm_tpm_Ch_Init(TPM1, 0, TPM_PWM_H, GPIOA, 12); //PTA12-> servo da base
    pwm_tpm_Ch_Init(TPM0, 0, TPM_PWM_H, GPIOC, 1);// PTC1-> servo do topo
   
    pwm_tpm_CnV(TPM1, 0, meio);
    k_msleep(2000);
    pwm_tpm_CnV(TPM0, 0, descanso);
    k_msleep(2000);



    while (1) {
        int err1 = adc_read(adc_dev, &sequence1);
        int err2 = adc_read(adc_dev, &sequence2);
        int err3 = adc_read(adc_dev, &sequence3);
        int err4 = adc_read(adc_dev, &sequence4);
       
        if (err1 !=0) {
            printk("Falha na leitura do ADC: %d\n", err1);
        }
        else if (err2 !=0) {
            printk("Falha na leitura do ADC: %d\n", err2);
        }
        else if (err3 !=0) {
            printk("Falha na leitura do ADC: %d\n", err3);
        }
        else if (err4 !=0) {
            printk("Falha na leitura do ADC: %d\n", err4);
        }
        else {
            int32_t Nmv = sample_buffer_N;
            int32_t Smv = sample_buffer_S;
            int32_t Lmv = sample_buffer_L;
            int32_t Omv = sample_buffer_O;
            adc_raw_to_millivolts (ADC_VREF_MV, ADC_GAIN, ADC_RESOLUTION, &Nmv);
            adc_raw_to_millivolts (ADC_VREF_MV, ADC_GAIN, ADC_RESOLUTION, &Smv);
            adc_raw_to_millivolts (ADC_VREF_MV, ADC_GAIN, ADC_RESOLUTION, &Lmv);
            adc_raw_to_millivolts (ADC_VREF_MV, ADC_GAIN, ADC_RESOLUTION, &Omv);




            uint32_t VN = Nmv-Smv;
            uint32_t VS = Smv;
            uint32_t VL = Lmv-Omv;
            uint32_t VO = Omv;



            printk("ADC Norte: %d (raw), %d mV\n", sample_buffer_N, VN);
            printk("ADC Sul: %d (raw), %d mV\n", sample_buffer_S, VS);
            printk("ADC Leste: %d (raw), %d mV\n", sample_buffer_L, VL);
            printk("ADC Oeste: %d (raw), %d mV\n", sample_buffer_O, VO);
            printk("\n");
//
            if(VL> VO+margem_LDR && movimento_b<max && movimento_b>zero) { //vai para oeste
                if(movimento_t> meio) {
                    movimento_b = movimento_b+ mexida;
                    pwm_tpm_CnV(TPM1, 0, movimento_b);
                    k_msleep(DELAY_LDR_MS);
                }
                else if(movimento_t< meio) {
                    movimento_b = movimento_b- mexida;
                    pwm_tpm_CnV(TPM1, 0, movimento_b);
                    k_msleep(DELAY_LDR_MS);
                }
            }



            else if(VO> VL+margem_LDR && movimento_b<max && movimento_b>zero) { //vai para leste
                if(movimento_t> meio) {
                    movimento_b = movimento_b- mexida;
                    pwm_tpm_CnV(TPM1, 0, movimento_b);
                    k_msleep(DELAY_LDR_MS);
                }
                else if(movimento_t< meio) {
                    movimento_b = movimento_b+ mexida;
                    pwm_tpm_CnV(TPM1, 0, movimento_b);
                    k_msleep(DELAY_LDR_MS);
                }
            }  
//          
            else if(VN> VS+margem_LDR) { //vai para sul
                movimento_t = movimento_t - mexida;
                pwm_tpm_CnV(TPM0, 0, movimento_t);
                k_msleep(DELAY_LDR_MS);
            }


            else if(VS> VN+margem_LDR) { //vai para norte
                movimento_t = movimento_t + mexida;
                pwm_tpm_CnV(TPM0, 0, movimento_t);
                k_msleep(DELAY_LDR_MS);
            }
            else {
                k_msleep(Delay_Leitura);
            }
            if(VN+VS> Noite) {
                pwm_tpm_CnV(TPM0, 0, descanso);
                k_msleep(1000);
                pwm_tpm_CnV(TPM1, 0, meio);
                k_msleep(Tempo_dormir);
                movimento_t = descanso;
                movimento_b = meio;
            }
        }
    }
    return 0;
}








// Sensor Norte: PTB 0- branco
// Sensor Sul: PTB 1 -cinza
// Sensor Leste: PTB 2 -amarelo
// Sensor Oeste: PTB 3 -dourado
// Motor base: PTA 12
// Motor topo: PTC 1
// Lembrar de definir posição dos LDR com base nas posições de descanso


