/*#########################################################
                         main.c
    Este c�digo utiliza a t�cnica SPWM para acionar as chaves de um inversor trif�sico.
    Universidade Federal do Cear� - Engenharia El�trica
    PIBIC 2020/2021: Desenvolvimento de uma bancada para acionamento de motores de indu��o trif�sicos
    Bolsista: Auro Gabriel Carvalho de Aramides
    Vers�o: 1
    Data: 08/2022
    Autor: Auro Gabriel Carvalho de Aramides
//#########################################################
//#########################################################
                    Descri��o dos pinos
    GPIO 6   - Sa�da ePWM 4A
    GPIO 7   - Sa�da ePWM 4B
    GPIO 8   - Sa�da ePWM 5A
    GPIO 9   - Sa�da ePWM 5B
    GPIO 10  - Sa�da ePWM 6A
    GPIO 11  - Sa�da ePWM 6B
    GPIO 15  - Sinal de alarme, respons�vel por gerar uma interrup��o externa quando necess�rio (XINT2)
    GPIO 104 - Pino que aciona o rel� que energiza a contactora que, por sua vez, energiza o circuito de pot�ncia.
    GPIO 105  - Pino que aciona o rel� que energiza a fonte chaveada
    GPIO 14 - 3V3 (pino usado para o opto acplador de prote��o)
    GPIO96 - eQEP1A
    GPIO97 - eQEP1B
    GPIO99 - eQEP1I
//#########################################################*/

#include "F28x_Project.h"
#include "math.h"

#define PORTADORA_FREQ 6000
#define MODULADORA_FREQ 60
#define NOS 200 //Number of Samples
#define Mi 0.8   // �ndice de modula��o
#define pi 3.14159265358979323846
int malha=0;
/////////////////////////
//Vari�vel do encoder
unsigned int Posicao_ADC = 10; //Leitura da velocidade no m�dulo eqep.
float Rotor_Posicao = 0; //Posi��o angular do rotor.
float Rotor_Posicao_Ant = 0; //Mem�ria da posi��o angular do rotor.
float delta_posicao =0; //Varia��o da posi��o angular do rotor.
float delta_posicao_1 =0; //Varia��o da posi��o angular do rotor (vari�vel auxiliar).
float Velo = 0; //Medida de velocidade mec�nica do rotor em RPM.
float Velo_avg =0; //Leitura filtrada de velocidade.
unsigned long Velo_ADC =0 ; //Medida digital da velocidade.
float Velo_aux = 0; //Vari�vel auxiliar para controle de velocidade.
float Velo_ant1 =0;//Valores passados para a velociddae do motor.
float w = 0;
float wa = 10;
float w_avg =0; //Velocidade angular depois do filtro.

////////////////////
Uint16 AlarmCount=0;                        // Alarm Counter
Uint16 index= 0;
Uint16 w1,w2,w3;
Uint16 TB_Prd;
Uint16 TB_Prescale;
Uint16 Comando_L_D;
Uint16 aux = 0;
Uint16 SPWM_State = 0;


float32 Converted_Voltage_P1=0;
float32 Converted_Voltage_P2=0;
float32 Converted_Voltage_P3=0;

float32 current_phase_1=0;
float32 current_phase_2=0;
float32 current_phase_3=0;


void Setup_GPIO(void);
void Setup_INTERRUPT(void);
void Setup_ePWM(void);
void Setup_ADC(void);

void Liga_Bancada(void);
void Desliga_Bancada(void);
void Stop_SPWM(void);

//Fun��es do Encoder
void Setup_eQEP(void);

void Set_ePWM_Frequency(uint32_t freq_pwm);


__interrupt void alarm_handler_isr(void);     // Alarm Handler interrupt service routine function prototype.

__interrupt void adca_isr(void);


void main(void){
//##########__INICIALIZA��O__#######################################################################

    // ATEN��O(%%%%%%% AURO: Mudei j� %%%%%%%%%%%%%%): A fun��o InitPeripheralClocks() (presente dentro da fun��o InitSysCtrl()) foi modificada
    // para habilitar apenas o clock dos perif�ricos que est�o sendo usados nesse c�digo. A saber:
    //  TBCLKSYNC; ePWM's. Seja consciente, economize energia :)

       InitSysCtrl();                          // PLL, WatchDog, enable Peripheral Clocks
       InitGpio();                             // Inicializa��o do GPIO
       DINT;                                   // Disable CPU interrupts
       InitPieCtrl();                          // Initialize PIE control registers to their default state.
       IER = 0x0000;                           // Disable CPU interrupts and clear all CPU interrupt flags:
       IFR = 0x0000;
       InitPieVectTable();                     // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).

//##########__CONFIGURA��ES INICIAIS__#######################################################################
    Set_ePWM_Frequency(PORTADORA_FREQ);                // Set the ePWM frequence in Hz. Min 193 hz. A frequ�ncia da portadora deve ser um m�ltiplo da moduladora.
                                                       //Para garantir um n�mero inteiro de pulsos por semiciclo. (RASHID).

    Setup_GPIO();                                      // Configura��o dos GPIOs
    Setup_ePWM();                                      // Abre todas as chaves
    Setup_ADC();
                                                       // Configura��o das interrup��es
    Setup_eQEP();

    EALLOW;                                             // Endere�o das rotinas de interrup��es
        PieVectTable.ADCA1_INT = &adca_isr;
        //PieVectTable.XINT2_INT  =  &alarm_handler_isr;
    EDIS;

//##########__CONFIGURA��O ADC_INT__#######################################################################
  EALLOW;

   PieCtrlRegs.PIEIER1.bit.INTx1   = 1 ;         // Habilita o PIE para interrup��o do ADC.
   AdcaRegs.ADCINTSEL1N2.bit.INT1E =1;
   EDIS;

   // IER |= M_INT1;

    EINT;                                          // Enable Global interrupt INTM
    ERTM;                                          // Enable Global realtime interrupt DBGM , UTILIZADO PARA ALTERAR O VALOR DOS REGISTRADORES EM TEMPO REAL.


//##########__CODIGO__#######################################################################
    while(1)
    {
        Comando_L_D != 0 ? Liga_Bancada():Desliga_Bancada(); // Uiliza o debug em tempo real para ligar ou desligar a bancada
                                                             // Alterando o valor da vari�vel Comando_L_D na janela de express�es
                                                             // do code composer studio.


    }

}


//##########__ADCA ISR___#######################################################################
__interrupt void adca_isr(){
        // Rotina ADC com 12 KHZ (frequ�ncia de amostragem do sinal senoidal da moduladora).
        if(malha==0){

            if(index  == 200 )
              index = 0;     //Limpa o Buffer.
            else
              index++;


            //Gera Moduladora Senoidal .
            w1 = (Uint16) (TB_Prd/2)*(1+Mi*__sin(__divf32(2*pi,NOS) * (float) index   ));
            w2 = (Uint16) (TB_Prd/2)*(1+Mi*__sin(__divf32(2*pi,NOS) * (float) index - 2*pi/3));
            w3 = (Uint16) (TB_Prd/2)*(1+Mi*__sin(__divf32(2*pi,NOS) * (float) index - 4*pi/3));



            EPwm4Regs.CMPA.bit.CMPA = w1;
            EPwm5Regs.CMPA.bit.CMPA = w2;
            EPwm6Regs.CMPA.bit.CMPA = w3;

        }

        Posicao_ADC  = EQep1Regs.QPOSCNT;

      //wa = (32.0*60.0/2048.0)/(EQep1Regs.QCPRD*6.4e-7);
      if(EQep1Regs.QEPSTS.bit.COEF == 0 && EQep1Regs.QEPSTS.bit.CDEF == 0){
          wa = (8.0*60.0/20)/(EQep1Regs.QCPRD*64.0/200.0e6);
      }else{
          EQep1Regs.QEPSTS.bit.COEF = 0;
          EQep1Regs.QEPSTS.bit.CDEF = 0;
      }


        //POSI�AO ANGULAR.
        Rotor_Posicao = ((float)Posicao_ADC)*DPI/2048.0;
        delta_posicao = Rotor_Posicao - Rotor_Posicao_Ant;

        //ROTOR PARADO ??
        if(delta_posicao == 0||Velo == 0){
            Velo_aux = Velo;
        }

        //VELOCIDADE NEGATIVA??
        if (delta_posicao <0 || Velo<0){
            delta_posicao = delta_posicao_1;
        }

        //VELOCIDADE POSITIVA.
        if (delta_posicao>0 || Velo>0){
            //DERIVADA DISCRETA DA POSI��O = VELOCIDADE.
            w = (float)((delta_posicao)/(0.000160));
            Velo = (float)(w*(60/(DPI)));
            Velo = wa;
            // Filtro passa baixa para f32f
            Velo_avg = Velo_avg + 0.000160*100.0*(Velo - Velo_avg);
            w_avg = (Velo_avg*DPI)/(60);

            //Velocidade medida em valores digitais.
            Velo_ADC = (int)(2.048*Velo_avg);
        }
        Rotor_Posicao_Ant = Rotor_Posicao;
        Velo_ant1 = Velo_avg;

    //IN�CIO DA MALHA DE CONTROLE.
    if (malha==1){
        //CAMPO ORIENTADO INDIRETO.
        float v_controle = Velo_avg;
        T = ((Llr+Lm)/Rr);
//        T = (Rr/(Llr+Lm));
        wsl = (ref_kq)/(T*ref_kd);
//        w_tot = wsl + 2*w_avg;
        w_tot = wsl + v_controle*DPI/60.0;

        theta_atual = ((160E-006)*w_tot) + theta_ant; // Integrador discreto.
        theta_ant = theta_atual;

        if(theta_atual > DPI){
            theta_atual = theta_atual -DPI;
            theta_ant = theta_atual;// Controle para manter Theta
        }
        if(theta_atual < 0){
            theta_atual = theta_atual +DPI;
            theta_ant = theta_atual;// Controle para manter Theta
        }

        // APLICA��O DAS TRANSFORMADAS DE EIXO DE CLARKE E PARK.
        // 2 - park.
        theta_rad = theta_atual;

        if(theta_rad > DPI){
            theta_rad = theta_rad - DPI;
        }
        if(theta_rad < -DPI){
            theta_rad = theta_rad + DPI;
        }

        I_d = 0.81649658092772603273242802490196*(ic*__cos(theta_rad) + ib*__cos(theta_rad - ((DPI)/3)) + ia*__cos(theta_rad -((2*DPI)/3)));
        I_q = 0.81649658092772603273242802490196*(-ic*__sin(theta_rad) - ib*__sin(theta_rad - ((DPI)/3)) - ia*__sin(theta_rad -((2*DPI)/3)));

        I_atual_d = I_d;
        I_atual_q = I_q;

        data1->id = I_d;
        data1->iq = I_q;

        //Correntes d e q em valores digitais.

        I_d_AD = (int)(1024*I_d) + 2048;
        I_q_AD = (int)(1024*I_q) + 2048;


        //MALHA DE VELOCIDADE.
        cont_velo ++;
        if(cont_velo==1225){
            //Refer�ncia degrau
            if(ref==1){
                ref_Velo = 1000;
                ref_Velo_AD = (int)(2.048*ref_Velo);
            }

            //Refer�ncia triangular
            if (ref==2){
                t = 0.096*cont_velo_aux;
                if(t<2){
                    ref_Velo = 100*(t) + 600 ;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=2){
                    ref_Velo = -100*(t) + 1000 ;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=4){
                    cont_velo_aux = 0;
                }
            }

            //Refer�ncia trapezoidal
            if (ref==3){
                t = 0.096*cont_velo_aux;
                if(t<1){
                    ref_Velo = 100*(t) + 600 ;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=1 && t<2){
                    ref_Velo = 800;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=2){
                    ref_Velo = -100*(t) + 1100 ;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=5 && t<6){
                    ref_Velo = 600;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=5 && t<6){
                    cont_velo_aux = 0;
                }
            }

            //Refer�ncia senoidal
            if(ref==4){
                t = 0.096*cont_velo_aux;
                ref_Velo = 600 + 200*__sin(DPI*0.25*t);
                cont_velo_aux++;
                if(t>=4){
                    cont_velo_aux = 0;
                }
            }
            if(ref==5){
                t = 0.096*cont_velo_aux;
                if(t >= 5){
                    ref_Velo = 1000;
                }else{
                    ref_Velo = 500;
                }
                cont_velo_aux++;
                if(t>=10){
                    cont_velo_aux = 0;
                }
                ref_Velo_AD = (int)(2.048*ref_Velo);
            }
            cont_velo = 0;

            //PI malha de velocidade
            double kp = 6.6667e-05;
            double ki = 6.6334e-05;

            erro_Velo = ref_Velo - v_controle;
            ref_kq = ref_kq_ant + ki*erro_Velo_ant1 + kp*erro_Velo;
            erro_Velo_ant1 = erro_Velo;
            ref_kq_ant = ref_kq;
        }
        ref_kd = 0.4;
        n_degrau++;
        if(n_degrau >= 6250*3){
            n_degrau = 0;
            if(iq == 0.5){
                iq = 0.7;
            }else{
                iq = 0.5;
            }
        }

        //REFER�NCIAS EIXO D E Q:
        ref_kd_AD = (int)(1024*ref_kd) + 2048;
        ref_kq_AD = (int)(1024*ref_kq) + 2048;

        //C�LCULO DOS ERROS
        erro_cd = ref_kd - I_d;
        erro_cq =  ref_kq - I_q;

        float kpi = 103, kii = 100.4;
        v_atual_d = v_d_ant + kpi*erro_cd - kii*erro_cd_ant1;
        v_atual_q = v_q_ant + kpi*erro_cq - kii*erro_cq_ant1;


        //Atualiza��o das vari�veis.
        erro_cd_ant1 = erro_cd;
        erro_cq_ant1 = erro_cq;

        I_q_ant = I_atual_q;
        v_d_ant = v_atual_d;
        v_q_ant = v_atual_q;

        //APLICA��O DAS TRANSFORMADAS INVERSAS DE CLARKE E PARK:
        Va =  0.81649658092772603273242802490196*(v_atual_d*__cos(theta_rad) - v_atual_q*__sin(theta_rad));
        Vb =  0.81649658092772603273242802490196*(v_atual_d*__cos(theta_rad - ((DPI)/3.0)) - v_atual_q*__sin(theta_rad - ((DPI)/3.0)));
        Vc =  0.81649658092772603273242802490196* (v_atual_d*__cos(theta_rad - ((2*DPI)/3.0)) - v_atual_q*__sin(theta_rad - ((2*DPI)/3.0)));

        //GANHO DE MODULA��O:
        Va = Va*16;
        Vb = Vb*16;
        Vc = Vc*16;

        //OFFSET:
        Va = Va + 4000;
        Vb = Vb + 4000;
        Vc = Vc + 4000;

        //SATURADOR:
        if (Va >= 8000){
            Va = 7950;
        }
        if (Va <= 0){
            Va = 50;
        }
        if (Vb >= 8000){
            Vb = 7950;
        }
        if (Vb <= 0 ){
            Vb = 50;
        }
        if (Vc >= 8000){
            Vc = 7950;
        }
        if (Vc <= 0){
            Vc = 50;
        }

        V_a = (Va - 4000)/16.0;
        V_b = (Vb - 4000)/16.0;
        V_c = (Vc - 4000)/16.0;

        //DacaRegs.DACVALS.all = (V_a*1024.0/1000.0) + 1024;
        //DacbRegs.DACVALS.all = (V_b*1024.0/1000.0) + 1024;

        //MODULA��O:
        EPwm1Regs.CMPA.bit.CMPA = Vc; // adjust duty for output EPWM1A
        EPwm2Regs.CMPA.bit.CMPA = Vb; // adjust duty for output EPWM2A
        EPwm3Regs.CMPA.bit.CMPA = Va; // adjust duty for output EPWM3A
        //SEQUENCIA CORRETA DE CIMA PRA BAIXO C,B,A
    }

    saida = 1;
    i_amostra++;
    data1->j = i_amostra;

     //   GpioDataRegs.GPBCLEAR.bit.GPIO34=1;

        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;    // Limpa as FLAGS provinientes do Trigger.
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  //


}
/*
// Transoforma o resultado decimal equivalente ao bin�rio da convers�o de cada fase em uma tens�o de -1,15 a 1,15 V
       Converted_Voltage_P1 = __divf32(3.0*AdcaResultRegs.ADCRESULT0,4096.0)-1.65;
       Converted_Voltage_P2 = __divf32(3.0*AdcbResultRegs.ADCRESULT1,4096.0)-1.65;
       Converted_Voltage_P3 = __divf32(3.0*AdccResultRegs.ADCRESULT2,4096.0)-1.65;

       /// * Adc Voltage Range = 0 : 3.0 V
       // * Sensor Voltage Range = 0.5 : 2.8 V
       // * Converted_Voltage_Px Voltage Range = -1.15 : 1.15 V


       //Calcula a corrente atrav�s da tens�o pela sensibilidade do sensor : 18.4 mV / A
        current_phase_1 =- __divf32(Converted_Voltage_P1,0.0184);
        current_phase_2 =- __divf32(Converted_Voltage_P2 ,0.0184);
        current_phase_3 =- __divf32(Converted_Voltage_P3,0.0184);

 *
 *
 */

//##########__ALARME ISR___#######################################################################
interrupt void alarm_handler_isr(void){

    //Alarme soou, codigo ficar� preso at� que o bot�o de "RE-DEBUG" seja pressinado no DSP.

    Stop_SPWM();                                      //Para o PWM e seta as sa�das (Abre as chaves).

    AlarmCount++;                                      // Contador do alarme

    while(1){

        GpioDataRegs.GPATOGGLE.bit.GPIO31 =1;       //LEDS do DSP para sinaliza��o do Alarme
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 =1;
        DELAY_US(400000);
    }



   //PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Reabilita interrup��es provenientes do alarme
}




void Set_ePWM_Frequency(uint32_t freq_pwm){
    if(freq_pwm < 763){                      //Minimum Value of frequency without dividing the EPWMCLK . Thus is necessary to divide it if we want less freq.
        TB_Prd =  (0x17D784)/(2*2*freq_pwm); //0x17D784 = 200 MHz / 128.
        TB_Prescale = 128;                   //Control variable to choose witch value will divide EPWMCLK.
    }
    else{                                    // Not necessary to divide EPWMCLK.
        TB_Prd =  (0xBEBC200)/(2*2*freq_pwm);//0xBEBC200 = 200 Mhz = EPWMCLk/1;
        TB_Prescale = 1;                     // Divide EPWMCLK by 1.
    }
}



void Liga_Bancada(void)
{

  if(aux == 0 )
  {
          EALLOW;

        Stop_SPWM();

        GpioDataRegs.GPDSET.bit.GPIO105 = 1;     // Liga fonte de controle.
        GpioDataRegs.GPBSET.bit.GPIO34=1;
        DELAY_US(2000000);
        GpioDataRegs.GPDSET.bit.GPIO104 = 1;    //Liga fonte de pot�ncia.


        while(1){if(SPWM_State){break;} }       // Espera at� que SPWM_State seja diferente de zero
                                               // Para liberar o PWM. Isso � feito alterando a vari�vel
                                               // SPWM_Satte em tempo real atrav�s da aba de exprss�es do DSP.

        Setup_GPIO();                         //Libera PWM.
        IER |= M_INT1;

        aux++;
        EDIS;

  }



}

void Desliga_Bancada(void)
{

    EALLOW;

    GpioDataRegs.GPDCLEAR.bit.GPIO104 = 1;  // Desliga fonte de pot�ncia

    DELAY_US(5000000);

    GpioDataRegs.GPDCLEAR.bit.GPIO105 = 1; // Desliga fonte de controle

    aux = 0;

    EDIS;
}

void Stop_SPWM(void){

    EALLOW;

    IER &= M_INT1;                          //Desabilita PWM assocaido a interrup��o do ADC.
    GpioCtrlRegs.GPAMUX1.all = 0;           // GPIO = GPIO
    GpioCtrlRegs.GPADIR.all = 0x00000FC0;
    GpioDataRegs.GPACLEAR.all = 0x00000FC0; // LOW (GPIO 6, 7, 8, 9, 10 e 11)
                                           // Abre todas as chaves.

    EDIS;
}


void Setup_GPIO(void){

EALLOW;
//##############################__FONTE 3V3__##############################

    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPAMUX1.bit.GPIO14  = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPADIR.bit.GPIO14   = 1;        // OUTPUT
    GpioDataRegs.GPASET.bit.GPIO14   = 1;        // HIGH

//##############################_GND CABO FLAT_##############################

    //GPIO26
    GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPADIR.bit.GPIO26   = 1;        // OUTPUT
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;        // LOW

    //GPIO66
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPCDIR.bit.GPIO66   = 1;        // OUTPUT
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;        // LOW

    //GPIO130
     GpioCtrlRegs.GPEGMUX1.bit.GPIO130 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPEDIR.bit.GPIO130   = 1;        // OUTPUT
     GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;        // LOW

     //GPIO131
     GpioCtrlRegs.GPEGMUX1.bit.GPIO131 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPEMUX1.bit.GPIO131 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPEDIR.bit.GPIO131   = 1;        // OUTPUT
     GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;        // LOW

     //GPIO63
     GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPBDIR.bit.GPIO63   = 1;        // OUTPUT
     GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1;        // LOW

     //GPIO64
     GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPCDIR.bit.GPIO64   = 1;        // OUTPUT
     GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;        // LOW

     //GPIO27
     GpioCtrlRegs.GPAGMUX2.bit.GPIO27= 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPADIR.bit.GPIO27   = 1;        // OUTPUT
     GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

     //GPIO25
      GpioCtrlRegs.GPAGMUX2.bit.GPIO25= 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPADIR.bit.GPIO25   = 1;        // OUTPUT
      GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

      //GPIO32
      GpioCtrlRegs.GPBGMUX1.bit.GPIO32 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPBDIR.bit.GPIO32   = 1;        // OUTPUT
      GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;        // LOW

      //GPIO19
      GpioCtrlRegs.GPAGMUX2.bit.GPIO19= 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPADIR.bit.GPIO19  = 1;        // OUTPUT
      GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

      //GPIO18
      GpioCtrlRegs.GPAGMUX2.bit.GPIO18= 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPADIR.bit.GPIO18   = 1;        // OUTPUT
      GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;

      //GPIO67
      GpioCtrlRegs.GPCGMUX1.bit.GPIO67 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPCDIR.bit.GPIO67   = 1;        // OUTPUT
      GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;        // LOW

      //GPIO111
      GpioCtrlRegs.GPDGMUX1.bit.GPIO111 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPDDIR.bit.GPIO111   = 1;        // OUTPUT
      GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;        // LOW

      //GPIO60
      GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPBDIR.bit.GPIO60   = 1;        // OUTPUT
      GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;        // LOW

      //GPIO22
      GpioCtrlRegs.GPAGMUX2.bit.GPIO22= 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPADIR.bit.GPIO22   = 1;        // OUTPUT
      GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

//##############################__ePWM 4, 5 e 6__###########################


       GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;
       GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
       GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;

       GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;
       GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
       GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;


      GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;
      GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
      GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;

      GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;
      GpioCtrlRegs.GPAMUX1.bit.GPIO9= 1;
      GpioCtrlRegs.GPAPUD.bit.GPIO9= 1;


      GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;
      GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
      GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;

      GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;
      GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
      GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;


//##############################__LEDS do DSP : ALARME__##############################

    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31  = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31  = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO31 =1;


    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;

    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34  = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO34 =1;

//##############################__ALARME__##############################

   GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;        // GPIO = GPIO (default)
   GpioCtrlRegs.GPAMUX1.bit.GPIO15  = 0;        // GPIO = GPIO (default)
   GpioCtrlRegs.GPADIR.bit.GPIO15   = 0;        // INPUT
   GpioCtrlRegs.GPAQSEL1.bit.GPIO15   = 2;          // XINT2 Qual using 6 samples
   GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 0xFF;      // Each sampling window
                                                  // is 510*SYSCLKOUT
  // InputXbarRegs.INPUT5SELECT = 15;               // GPIO 15 is XINT2


//#############################__Acionamento e desligamento da bancada__###########

   //Base do transistor que aciona o rel� da fonte de controle.
   GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;
   GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;
   GpioCtrlRegs.GPDDIR.bit.GPIO105 = 1;
   //Base do transistor que aciona o rel� da fonte de pot�ncia.
   GpioCtrlRegs.GPDGMUX1.bit.GPIO104 = 0;
   GpioCtrlRegs.GPDGMUX1.bit.GPIO104 = 0;
   GpioCtrlRegs.GPDDIR.bit.GPIO104 = 1;


//##################################__ADC__##########3

   GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0;



EDIS;


}

void Setup_ADC(){

        Uint16 acqps;

        // Configura��es m�nimas, consultar datasheet pag 105.

        if( ADC_RESOLUTION_12BIT  == AdcaRegs.ADCCTL2.bit.RESOLUTION)
            acqps = 14;
        else
            acqps = 63;

        EALLOW;

      //##################################################################### ADC A ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_A =1;   // Habilita o clock do m�dulo A do ADC.
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

        AdcSetMode(ADC_ADCA,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Gera interrup��o um ciclo de clock antes do EOC.
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;   // Energiza o ADC A .
        DELAY_US(1000);  // 1ms de delay para ligar o m�dulo do ADC.

        // SOC and INTERRUPT config
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3; // ADCINA3 - PINO 26 (J3).
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; // 64 SYSCLK cycles to charge the capacitor. Recomendado no Datasheet , pag 105.



        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // EOC DISPARA o  ADCINT1;
        AdcaRegs.ADCINTSEL1N2.bit.INT1E =0;   // Desabilita interrup��es do ADC;
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 =1; //Make sure the INT1 flag is cleared.



        //##################################################################### ADC B ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_B = 1;   // Habilita o clock do m�dulo B do ADC.
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6;  // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

        AdcSetMode(ADC_ADCB,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdcbRegs.ADCCTL1.bit.INTPULSEPOS=1; // Gera interrup��o um ciclo de clock antes do EOC.
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // Energiza o ADC B.
        DELAY_US(1000);  // 1ms de delay para ligar o m�dulo do ADC.

        // SOC config
        AdcbRegs.ADCSOC1CTL.bit.CHSEL =3 ; // ADCINB3 - PINO 25 (J3).
        AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;


        //##################################################################### ADC C ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
        AdccRegs.ADCCTL2.bit.PRESCALE = 6;  // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

        AdcSetMode(ADC_ADCC,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdccRegs.ADCCTL1.bit.INTPULSEPOS=1; // Gera interrup��o um ciclo de clock antes do EOC.
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // Energiza o ADC C.
        DELAY_US(1000);  // 1ms de delay para ligar o m�dulo do ADC.

        // SOC config
        AdccRegs.ADCSOC2CTL.bit.CHSEL =3 ; // ADCINC3 - PINO 24 (J3).
        AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps;



        EDIS;

}


void Setup_ePWM(void){

    EALLOW;                    //###### LEMBRAR DE AJUSTAR O FED E RED PARA OS PREESCALES DO TBCLOCK.

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;           // DISABLE TBCLK for ePWM configuration.

//##########__EPWM4__###################################################################

    EPwm4Regs.TBPRD = TB_Prd;                       // Set timer period TBPR = (EPWMCLK ) / ( 2 x 2 x freq_pwm) for up-down count mode.
    EPwm4Regs.CMPA.bit.CMPA = 0;                    // Clear CMPA
    EPwm4Regs.TBPHS.bit.TBPHS = 0;                  // Regsitrador de fase zerado.
    EPwm4Regs.TBCTL.bit.SYNCOSEL =TB_CTR_ZERO  ;    // Sincroniza as fases em TBPRD = 0.
    EPwm4Regs.TBCTR = 0x0000;                       // Limpa o contador.
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Configura a portadora para o modo sim�trico(up-down).
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Desabilita o carregamento das fases.

    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        //TBCLK = EPWMCLK/(HSPCLKDIV*CLKDIV)
    if(TB_Prescale == 1)         EPwm4Regs.TBCTL.bit.CLKDIV = 0;      // EPWMCLK/1.
    else if (TB_Prescale == 128) EPwm4Regs.TBCTL.bit.CLKDIV = 7;      // EPWMCLK/128.

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;       //Habilita o shadow como um buffer duplo.
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; /* Carrega os registradores nos eventos TBCTR = ZERO  e TBCTR = PRD.
                                                         Necess�rio pois a senoide apresenta valores diferentes em CAU e CAD.
                                                      */

    // Configura��o do Action Qualifier para gera��o do SPWM.
    EPwm4Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active Low complementary to use EPWM4A complementary to EMPW4B.
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // Habilita o m�dulo de Dead-Band.
    EPwm4Regs.DBFED.bit.DBFED = 400;                // FED = 350 TBCLKs (4.0 uS)
    EPwm4Regs.DBRED.bit.DBRED = 400;                // RED = 350 TBCLKs (4.0 uS)


    //##########__EPWM5__##########################################################################

    EPwm5Regs.TBPRD = TB_Prd;
    EPwm5Regs.CMPA.bit.CMPA = 0;
    EPwm5Regs.TBPHS.bit.TBPHS = 0;
    EPwm5Regs.TBCTL.bit.SYNCOSEL =TB_CTR_ZERO ;
    EPwm5Regs.TBCTR = 0x0000;
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

    if (TB_Prescale == 1)         EPwm5Regs.TBCTL.bit.CLKDIV = 0;      // EPWMCLK/1
    else if (TB_Prescale == 128) EPwm5Regs.TBCTL.bit.CLKDIV = 7;       // EPWMCLK/128

    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;


    EPwm5Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBFED.bit.DBFED = 400;
    EPwm5Regs.DBRED.bit.DBRED =400;

//##########__EPWM6__##########################################################################

    EPwm6Regs.TBPRD = TB_Prd;
    EPwm6Regs.CMPA.bit.CMPA = 0;
    EPwm6Regs.TBPHS.bit.TBPHS = 0;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm6Regs.TBCTR = 0x0000;
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

    if(TB_Prescale == 1)        EPwm6Regs.TBCTL.bit.CLKDIV = 0;
    else if (TB_Prescale == 128)EPwm6Regs.TBCTL.bit.CLKDIV = 7;

    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm6Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBFED.bit.DBFED = 400;
    EPwm6Regs.DBRED.bit.DBRED = 400;

//##########__EPWM1__##########################################################################

        // PWM sendo utilizado como TIMER , gerando SOC a 12 Khz.
       // Frequencia de amostragem deve ser um multiplo de 60.

     EPwm1Regs.TBPRD = 4165 ;  // Sampling at 12 Khz / TBPRD = TPWM/TBCLK - 1 -> FAZER O CALCULO COM A FREQ !!
     EPwm1Regs.TBPHS.bit.TBPHS = 0;
     EPwm1Regs.TBCTR = 0X0000;
     EPwm1Regs.TBCTL.bit.CTRMODE= TB_COUNT_UP; // Configura como up count mode.

     // Condigura��es padr�es para gerar TBCLK = 50 Mhz  ( EPWMCLK = SYSCLKOUT/2 = 100 Mhz , TBCLK = EPWMCLK/(HSPCLKDIV * CLKDIV) );
     EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;
     EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;

     EPwm1Regs.ETSEL.bit.SOCAEN =1;  // Enable SOC on A group
     EPwm1Regs.ETSEL.bit.SOCASEL=ET_CTR_PRD ;  // Dispara o SOC em CTR = PRD.
     EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;     // Dispara o SOC no primeiro evento.

     CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;                // ENABLE TBCLKs

       EDIS;

}
void Setup_eQEP(){

    EALLOW;


        //EQEP.
        InitEQep1Gpio();

        GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 0xF);
        GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
        GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 0xF);
        GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);


    //Configura o EQep1:

       EQep1Regs.QUPRD = 1;            // Unit Timer for 100Hz at 200 MHz
                                                 // SYSCLKOUT


       EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode

       EQep1Regs.QEPCTL.bit.FREE_SOFT = 0x2;//QEP Control =QEPCTL
       EQep1Regs.QEPCTL.bit.PCRM = 01;       // PCRM=01 -> Position counter reset on Max position counter
       EQep1Regs.QEPCTL.bit.UTE = 1;        // Unit Timeout Enable
       EQep1Regs.QEPCTL.bit.QCLM = 1;       // Latch on unit time out



       EQep1Regs.QPOSMAX = 0x20;            //0xffff=65535 contagens de quadratura
       EQep1Regs.QDECCTL.bit.SWAP = 1;      //troca o sentido da contagem

       EQep1Regs.QCAPCTL.bit.UPPS = 3;      // 1/8 for unit position
       EQep1Regs.QCAPCTL.bit.CCPS = 6;      // 1/64 for CAP clock

       EQep1Regs.QCAPCTL.bit.CEN = 1;       // QEP Capture Enable
       EQep1Regs.QEPCTL.bit.QPEN = 1;       //QEP enable

       //EQep1Regs.QEINT.bit.UTO = 1;       // 400 Hz interrupt for speed estimation
       EDIS;

    //################## Auro: Mudei os valores para o que a gente usa ############
    //
    // EXTRACTED FROM FILE:    Example_posspeed.c
    //
    // TITLE:   Pos/speed measurement using EQEP peripheral
    //
    // DESCRIPTION:
    //
    // This file includes the EQEP initialization and position and speed
    // calculation functions called by Eqep_posspeed.c.  The position and speed
    // calculation steps performed by POSSPEED_Calc() at  SYSCLKOUT =  200 MHz are
    // described below:

    //#############################################################################
    //  ###### Encoder de 5 pulsos por volta -> 20 de quadratura #################
    //#############################################################################

    // 1. This program calculates: **theta_mech**
    //
    //    theta_mech = QPOSCNT/mech_Scaler = QPOSCNT/20, where 20 is the number
    //                 of counts in 1 revolution.


    //                (20/4 = 5 line/rev. quadrature encoder)
    //

    // 2. This program calculates: **theta_elec**
    //
    //    theta_elec = (# pole pairs) * theta_mech = 2*QPOSCNT/20
    //


    // 3. This program calculates: **SpeedRpm_fr**
    //
    //    SpeedRpm_fr = [(x2-x1)/20]/T   - Equation 1

    //    Note (x2-x1) = difference in number of QPOSCNT counts. Dividing (x2-x1)
    //    by 20 gives position relative to Index in one revolution.


    // If base RPM  = 1800 rpm:   1800 rpm = [(x2-x1)/20]/10ms   - Equation 2

    //                                     = [(x2-x1)/20]/(.01s*1 min/60 sec)
    //                                     = [(x2-x1)/20]/(1/6000) min  - Equation 2

    //                         max (x2-x1) = 20 counts, or 1 revolution in 10 ms


    // If both sides of Equation 2 are divided by 1800 rpm, then:
    //                   1 = [(x2-x1)/20] rev./[(1/6000) min * 1800rpm]

    //                   Because (x2-x1) must be <20 (max) for QPOSCNT increment,
    //                   (x2-x1)/20 < 1 for CW rotation

    //                   And because (x2-x1) must be >-20 for QPOSCNT decrement,
    //                   (x2-x1)/1800>-1  for CCW rotation (CCW= Counter Clockwise)

    //                   speed_fr = [(x2-x1)/20]/[(1/6000) min * 1800rpm]
    //                            = (x2-x1)/2     - Equation 3


    // To convert speed_fr to RPM, multiply Equation 3 by 1800 rpm
    //                   SpeedRpm_fr = 1800rpm *(x2-x1)/2  - Final Equation



    // 2. **min rpm ** = selected at 10 rpm based on [CCPS] prescaler options
    //    available (128 is greatest)


    // 3. **SpeedRpm_pr**

    //    SpeedRpm_pr = X/(t2-t1)  - Equation 4

    //    where X = QCAPCTL [UPPS]/20 rev. (position relative to Index in
    //                                        1 revolution)
    // If  max/base speed = 1800 rpm:
    //               1800 = (8/20)/[(t2-t1)/(200MHz/64)]

    //          where 8 = QCAPCTL [UPPS] (Unit timeout - once every 8 edges)
    //          where 64 = QCAPCTL [CCPS]

    //            8/20 = position in 1 revolution (position as a fraction
    //                                                of 1 revolution)

    //   t2-t1/(200MHz/64), t2-t1= # of QCAPCLK cycles, and
    //      QCAPCLK cycle = 1/(200MHz/64)
//////                    = QCPRDLAT   #################################################


    // So:       1800 rpm = [UPPS(200MHz/CCPS)*60s/min]/[20(t2-t1)]
    //

    //           1800 rpm = [8*(200MHz/64)*60s/min]/[20(t2-t1)]
    //           t2-t1 = [8*(200MHz/64)*60s/min]/(20*1800rpm)  - Equation 5
    //                    ~= 416.66 CAPCLK cycles = maximum (t2-t1) = SpeedScaler
       //                   52.0825
    //
    // Divide both sides by (t2-t1), and:
    //            1 = 32/(t2-t1) = [8(200MHz/64)*60 s/min]/(20*1800rpm)]/(t2-t1)

    //       Because (t2-t1) must be < 416.66 for QPOSCNT increment:
    //               416.66/(t2-t1) < 1 for CW rotation
    //       And because (t2-t1) must be >-416.66 for QPOSCNT decrement:
    //                416.66/(t2-t1)> -1 for CCW rotation
    //
    //       eed_pr = 416.66/(t2-t1)
    //             or [8(200MHz/64)*60 s/min]/(20*1800rpm)]/(t2-t1) - Equation 6
    //
    // To convert speed_pr to RPM:
    // Multiply Equation 6 by 1800rpm:
    // SpeedRpm_fr  = 1800rpm * [8*(200MHz/64)*60 s/min]/[20*1800rpm*(t2-t1)]
    //              = [8(200MHz/64)*60 s/min]/[20*(t2-t1)]
    //              or [(8/20)rev * 60 s/min]/[(t2-t1)(QCPRDLAT)]-Final Equation
    //
    // More detailed calculation results can be found in the Example_freqcal.xls
    // spreadsheet included in the example folder.
    //
    // Olhar o exemplo da texas citado no "title" para entender mais coisas.
    //###########################################################################


}
