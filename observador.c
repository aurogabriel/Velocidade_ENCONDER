/*
 * observador.c
 * Autores: Jucelino Taleires, Rubem Bezerra
 * Laboratório de Sistemas Motrizes
 */

#include<math.h>
#include "F28x_Project.h"
#include "params.h"
#include "datastruct.h"
#include <stdint.h>

#define DPI 6.28318530717958647692

msg_data *data2 = (void *)(uint32_t)0x03FC00;







#if defined(MRAS)

// Declarações dos anteriores para ref_mras
float xka1 = 0, xkb1 = 0, dphia_k1 = 0, dphib_k1 = 0, prodk1 = 0, wrk1 = 0, wrf = 0, wr = 0, aux_alpha = 0, aux_beta = 0, phir_alpha_rt = 0, phir_beta_rt = 0;

float aux_alpha2 = 0, aux_alpha3 = 0, aux_beta2 = 0, aux_beta3 = 0;

float ref_OBS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){
    // Transformação de clarke de v
    // Mudanças: valpha e vbeta trocados e mecanismo de adaptação
    float valpha = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float vbeta = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);
//    DacaRegs.DACVALS.all = (vbeta*2000.0/250.0);
//    DacbRegs.DACVALS.all = (valpha*2000.0/250.0);

    // Transformação de clarke de i
    float ialpha = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ibeta = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);
//    DacaRegs.DACVALS.all = (ialpha*2000.0/1.0) + 2000;
//    DacbRegs.DACVALS.all = (ibeta*2000.0/1.0) + 2000;
    data2->ia = ialpha;
    data2->ib = ibeta;
    // Variável auxiliar para integração
    float xka = (valpha - Rs*ialpha);
    float xkb = (vbeta - Rs*ibeta);

    // Integração pura
//    aux_alpha = aux_alpha + (Ts/2)*xka + (Ts/2)*xka1;
//    aux_beta  = aux_beta  + (Ts/2)*xkb + (Ts/2)*xkb1;
//
//    float phir_alpha_st = (Lr/Lm)*(aux_alpha - sigma*Ls*ialpha );
//    float phir_beta_st  = (Lr/Lm)*(aux_beta  - sigma*Ls*ibeta );

    // Integração do fluxo com correção
    float g1 = 9.993602e-01, g2 = 3.198976e-06, g3 = 9.993602e-01, g4 = 6.397952e-04;
    aux_alpha = g1*aux_alpha + g2*xka;
    aux_beta  = g1*aux_beta  + g2*xkb;

    if(aux_alpha3 >= 0.5) aux_alpha3 = 0.5;
    if(aux_alpha3 <= -0.5) aux_alpha3 = -0.5;
    if(aux_beta3 >= 0.5) aux_beta3 = 0.5;
    if(aux_beta3 <= -0.5) aux_beta3 = -0.5;

    aux_alpha2 = g3*aux_alpha2 + g4*aux_alpha3;
    aux_beta2  = g3*aux_beta2  + g4*aux_beta3;

    aux_alpha3 = aux_alpha + aux_alpha2;
    aux_beta3  = aux_beta  + aux_beta2;


    float phir_alpha_st = (Lr/Lm)*(aux_alpha3 - sigma*Ls*ialpha );
    float phir_beta_st  = (Lr/Lm)*(aux_beta3  - sigma*Ls*ibeta );
//    DacaRegs.DACVALS.all = (phir_alpha_st*2000.0/2.0) + 2000;
//    DacbRegs.DACVALS.all = (phir_beta_st*2000.0/2.0) + 2000;
    //DacaRegs.DACVALS.all = phir_beta_st * 1024.0 / 2.0 + 1024;
    //float pos = atan2(phir_alpha_st, phir_beta_st);
    //DacbRegs.DACVALS.all = pos * 1024.0 / 7 + 1024;

    // Modelo Adaptativo
    float dphir_alpha_rt = -(1/Tr)*phir_alpha_rt - wr*phir_beta_rt  + (Lm/Tr)*ialpha;
    float dphir_beta_rt  = -(1/Tr)*phir_beta_rt  + wr*phir_alpha_rt + (Lm/Tr)*ibeta;
    phir_alpha_rt  = (phir_alpha_rt + (Ts/2)*dphir_alpha_rt + (Ts/2)*dphia_k1);
    phir_beta_rt   = phir_beta_rt  + (Ts/2)*dphir_beta_rt  + (Ts/2)*dphib_k1;

    data2->phia = phir_alpha_rt;
    data2->phib = phir_beta_rt;

    // Mecanismo de adaptação
    float prodk = -(phir_beta_st*phir_alpha_rt - phir_alpha_st*phir_beta_rt);
    //float prodk = -(phir_beta_st*phir_beta_rt - phir_alpha_st*phir_alpha_rt);

    wr = (wrk1 + fpb_b*prodk1 + fpb_a*prodk);
    //wr = Velo;

    //DacaRegs.DACVALS.all = 1*(phir_alpha_rt * 2048.0 / 5.0) + 1024; // Amarelo
    //DacbRegs.DACVALS.all = 1*(phir_alpha_st * 2048.0 / 5.0) + 1024; // Azul

    // Filtro passa-baixas
    wrf = (wrf + Ts*10*wr)/(1+Ts*10);
    //DacbRegs.DACVALS.all = wrf * 1024.0 / 1000.0 + 1024;

    // Atualização das variáveis
    xka1 = xka;
    xkb1 = xkb;

    dphia_k1 = dphir_alpha_rt;
    dphib_k1 = dphir_beta_rt;

    prodk1 = prodk;
    //wr = Velo*DPI*p/60.0;
    wrk1 = wr;
    //return pos; -ref_OBS(vc_obs, vb_obs, va_obs, ia, ib, ic, 0)*0.79 + 200
    return -(60.0/DPI)*(1.0/p)*wrf*0.79 + 200.0;  // conversão de rad/s para rpm
}






#elif defined(SMO)

// Declaracoes para o observador SMO
float dialpha_est = 0, dibeta_est = 0, ialpha_est = 0, ibeta_est = 0, qalpha = 0, qbeta = 0, salpha = 1, sbeta = 1, salphak1 = 0, sbetak1 = 0, dflux_est_alpha = 0, dflux_est_beta= 0, flux_est_alpha= 0, flux_est_beta= 0, dqalpha = 0, dqbeta = 0;

float y1alpha = 0, y1beta = 0, y2alpha = 0, y2beta = 0, zalpha = 0, zbeta = 0;  // variáveis para integração das correntes
float wfiltro = 0, dfilt = 0;

float int_sa = 0;

float valores_D[10], media_D;
int i_D = 0;

float ref_OBS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){ // Observador de fluxo rotórico e de velocidade por modos deslizantes



// Transformação de eixos de referência (invariante em amplitude)
    float vbeta = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float valpha = -(2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    float ialpha = (2.0/3.0)*(ia_in -0.5*ib_in - 0.5*ic_in);
    float ibeta  = -(2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);
    data2->ia = ialpha;
    data2->ib = ibeta;

    // Observador de corrente
    // Derivadas das correntes em alpha/beta
    dialpha_est = qalpha - k1*ialpha_est + k2*valpha;
    dibeta_est  = qbeta  - k1*ibeta_est  + k2*vbeta;


    // Integração da corrente
    ialpha_est = ialpha_est + Ts*(dialpha_est);
    ibeta_est  = ibeta_est  + Ts*(dibeta_est);

//    DacaRegs.DACVALS.all = 1*(ialpha_est * 2048.0 / 5.0) + 1024; // Amarelo
//    DacbRegs.DACVALS.all = 1*(ibeta_est * 2048.0 / 5.0) + 1024; // Azul

    // Cálculo das funções "s" em alpha/beta
//     salpha = ialpha_est - ialpha;
//     sbeta  = ibeta_est  - ibeta;

        salpha = (ialpha_est - ialpha);
        sbeta  = (ibeta_est  - ibeta);

    //DacaRegs.DACVALS.all = 1*(qalpha * 2048.0 / 10000.0) + 1024; // Amarelo
    //DacbRegs.DACVALS.all = 1*(qbeta * 2048.0 / 10000.0) + 1024; // Azul
    //int_sa += Ts*(salpha-salphak1);

    // Cálculo de dq
    dqalpha = - (1/Ts)*( (1+Ts*gamma1)*salpha - salphak1);
    dqbeta  = - (1/Ts)*( (1+Ts*gamma2)*sbeta - sbetak1 );

//    dqalpha = ((1+Ts*gamma1)*salpha - salphak1);
//    dqbeta =  (sbeta - sbetak1);

//     Cálculo de q
    qalpha = qalpha - (1/Ts)*( (1+Ts*gamma1)*salpha - salphak1);
    qbeta  = qbeta  - (1/Ts)*( (1+Ts*gamma2)*sbeta - sbetak1 );

    //qalpha += dqalpha;
    //qbeta += dqbeta;


//    DacaRegs.DACVALS.all = 1*(qalpha * 1.0 / 200.0) + 1024; // Amarelo
//    DacbRegs.DACVALS.all = 1*(qbeta * 1.0 / 200.0) + 1024; // Azul


    // Cálculo das derivadas dos fluxos
    dflux_est_alpha = - qalpha/gamma;
    dflux_est_beta  = - qbeta/gamma;


    // Integração dos fluxos com correção
//    float m1 = 9.996801e-01, m2 = 3.199488e-06, m3 = 9.996801e-01, m4 = 3.199488e-04, sat =0.1; // wc = 100
    float m1 = 9.996001e-01, m2 = 3.199360e-06, m3 = 9.996001e-01, m4 = 3.999200e-04, sat = 0.2; // wc = 125


                y1alpha = m1*y1alpha +  m2*dflux_est_alpha;
                y1beta  = m1*y1beta  + m2*dflux_est_beta;
                zalpha = flux_est_alpha;
                zbeta = flux_est_beta;
                if(flux_est_alpha >= sat) zalpha = sat;
                if(flux_est_alpha <= -sat) zalpha = -sat;
                if(flux_est_beta >= sat) zbeta = sat;
                if(flux_est_beta <= -sat) zbeta = -sat;

                y2alpha = m3*y2alpha + m4*zalpha;
                y2beta  = m3*y2beta  + m4*zbeta;

                flux_est_alpha = (y1alpha + y2alpha);
                flux_est_beta  = (y1beta  + y2beta);
                data2->phia = flux_est_alpha;
                data2->phib = flux_est_beta;

//                DacaRegs.DACVALS.all = (flux_est_alpha*2000.0/1.0) + 2000;
//                DacbRegs.DACVALS.all = (flux_est_beta*2000.0/1.0) + 2000;
////     Integração dos fluxos
//    flux_est_alpha = 9.996801e-01*flux_est_alpha + 3.199488e-06*(dflux_est_alpha); // usando FPB (wc = 100)
//    flux_est_beta  =  9.996801e-01*flux_est_beta  + 3.199488e-06*(dflux_est_beta);

//    flux_est_alpha  = flux_est_alpha  + Ts*(dflux_est_alpha);  // integrador puro
//    flux_est_beta  = flux_est_beta  + Ts*(dflux_est_beta);



//    DacaRegs.DACVALS.all = 1*(flux_est_alpha * 2048.0 / 5.0) + 1024; // Amarelo
//    DacbRegs.DACVALS.all = 1*(flux_est_beta * 2048.0 / 5.0) + 1024; // Azul

    // Estimador de velocidade
    float wr = ((flux_est_alpha*dflux_est_beta - flux_est_beta*dflux_est_alpha) - Lm*(Rr/Lr)*(ibeta_est*flux_est_alpha - ialpha_est*flux_est_beta));
    float D = (flux_est_beta*flux_est_beta + flux_est_alpha*flux_est_alpha);
//    if(D <= 0.01) D = 0.01;

//     float D = (1/(gamma*gamma))*(qalpha*qalpha + qbeta*qbeta) + (Lm/gamma)*(qalpha*dialpha_est + qbeta*dibeta_est);
//     float wr = (1/(gamma*D))*( (-qbeta/gamma - Lm*dibeta_est)*dqalpha + (qalpha/gamma + Lm*dialpha_est)*dqbeta);
//     wfiltro = 9.968051e-01*wfiltro + 3.194885e-06*wr;



    // Atualização das variáveis
    salphak1 = salpha;
    sbetak1 = sbeta;

    // Saída do observador
//    wr = wr/D;
//    wfiltro = m1*wfiltro + m2*wr;


//    dfilt =  9.999968e-01*dfilt + 3.199995e-06*(D); // wc = 1
    dfilt =  9.999680e-01*dfilt + 3.199949e-05*(D); // wc = 10
//    dfilt =  9.999360e-01*dfilt + 6.399795e-05*(D); // wc = 20
//    dfilt =  9.999040e-01*dfilt + 9.599539e-05*(D); // wc = 30
//    dfilt =  9.998720e-01*dfilt + 1.279918e-04*(D); // wc = 40
//    dfilt =  9.998400e-01*dfilt + 1.599872e-04*(D); // wc = 50

//     wr = wr/D;
     wfiltro = 9.999680e-01*wfiltro + 3.199949e-06*(wr);

//     DacaRegs.DACVALS.all = dfilt*10;
//     DacbRegs.DACVALS.all = wfiltro;

//     wfiltro = wr;

//    DacbRegs.DACVALS.all = (wfiltro * (2048.0 / 20.0)) + 1024; // Azul
//    DacaRegs.DACVALS.all = ((dfilt) * 2048.0 / 1.0) + 1024; // Amarelo


//    DacaRegs.DACVALS.all = ((wfiltro/dfilt) *2048.0 / 20.0) + 1024; // Amarelo


//    valores_D[i_D] = D;
//    i_D++;
//    i_D %= 10;
//    int i;
//    media_D = 0;
//    for(i = 0; i < 10; i++){
//        media_D = media_D + valores_D[i]/10.0;
//    }
    return -(60.0/DPI)*wfiltro/dfilt*4.5 + 70;
}

#elif defined(MRAS_SMO)


// Declarações dos anteriores para ref_mras
float xka1 = 0, xkb1 = 0, dphia_k1 = 0, dphib_k1 = 0, prodk1 = 0, wrk1 = 0, wrf = 0, wr = 0, aux_alpha = 0, aux_beta = 0, phir_alpha_rt = 0, phir_beta_rt = 0;

float aux_alpha2 = 0, aux_alpha3 = 0, aux_beta2 = 0, aux_beta3 = 0;

float dialpha_est = 0, dibeta_est = 0, ialpha_est = 0, ibeta_est = 0, qalpha = 0, qbeta = 0, salpha = 1, sbeta = 1, salphak1 = 0, sbetak1 = 0, dflux_est_alpha = 0, dflux_est_beta= 0, flux_est_alpha= 0, flux_est_beta= 0, dqalpha = 0, dqbeta = 0;

float y1alpha = 0, y1beta = 0, y2alpha = 0, y2beta = 0, zalpha = 0, zbeta = 0;  // variáveis para integração das correntes


float ref_OBS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in, float Velo){
    // Transformação de clarke de v
    // Mudanças: valpha e vbeta trocados e mecanismo de adaptação
    float valpha = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float vbeta = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);
    //    DacaRegs.DACVALS.all = (vbeta*2000.0/250.0);
    //    DacbRegs.DACVALS.all = (valpha*2000.0/250.0);

    // Transformação de clarke de i
    float ialpha = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ibeta = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    float valpha_smo = -vbeta;
    float vbeta_smo = valpha;
    float ialpha_smo = ialpha;
    float ibeta_smo = -ibeta;
    //DacaRegs.DACVALS.all = (ialpha*2000.0/1.0) + 2000;
    //DacbRegs.DACVALS.all = (ibeta*2000.0/1.0) + 2000;
    dialpha_est = qalpha - k1*ialpha_est + k2*valpha_smo;
    dibeta_est  = qbeta  - k1*ibeta_est  + k2*vbeta_smo;


    // Integração da corrente
    ialpha_est = ialpha_est + Ts*(dialpha_est);
    ibeta_est  = ibeta_est  + Ts*(dibeta_est);

//        DacaRegs.DACVALS.all = 1*(ialpha * 2000.0 / 1.0) + 2000; // Amarelo
//        DacbRegs.DACVALS.all = 1*(ibeta * 2008.0 / 1.0) + 2000; // Azul

        // Cálculo das funções "s" em alpha/beta
    //     salpha = ialpha_est - ialpha;
    //     sbeta  = ibeta_est  - ibeta;

    salpha = (ialpha_est - ialpha_smo);
    sbeta  = (ibeta_est  - ibeta_smo);

    //DacaRegs.DACVALS.all = 1*(qalpha * 2048.0 / 10000.0) + 1024; // Amarelo
    //DacbRegs.DACVALS.all = 1*(qbeta * 2048.0 / 10000.0) + 1024; // Azul
    //int_sa += Ts*(salpha-salphak1);

    // Cálculo de dq
    dqalpha = - (1/Ts)*( (1+Ts*gamma1)*salpha - salphak1);
    dqbeta  = - (1/Ts)*( (1+Ts*gamma2)*sbeta - sbetak1 );

    //    dqalpha = ((1+Ts*gamma1)*salpha - salphak1);
    //    dqbeta =  (sbeta - sbetak1);

    //     Cálculo de q
    qalpha = qalpha - (1/Ts)*( (1+Ts*gamma1)*salpha - salphak1);
    qbeta  = qbeta  - (1/Ts)*( (1+Ts*gamma2)*sbeta - sbetak1 );

    //qalpha += dqalpha;
    //qbeta += dqbeta;


    //    DacaRegs.DACVALS.all = 1*(qalpha * 1.0 / 200.0) + 1024; // Amarelo
    //    DacbRegs.DACVALS.all = 1*(qbeta * 1.0 / 200.0) + 1024; // Azul


    // Cálculo das derivadas dos fluxos
    dflux_est_alpha = - qalpha/gamma;
    dflux_est_beta  = - qbeta/gamma;


    // Integração dos fluxos com correção
    //    float m1 = 9.996801e-01, m2 = 3.199488e-06, m3 = 9.996801e-01, m4 = 3.199488e-04, sat =0.1; // wc = 100
//    float m1 = 9.996001e-01, m2 = 3.199360e-06, m3 = 9.996001e-01, m4 = 3.999200e-04, sat = 0.1; // wc = 125
    float m1 = 9.996801e-01, m2 = 3.199488e-06, m3 = 9.996801e-01, m4 = 3.199488e-04, sat = 0.2; // wc = 125

    y1alpha = m1*y1alpha +  m2*dflux_est_alpha;
    y1beta  = m1*y1beta  + m2*dflux_est_beta;
    zalpha = flux_est_alpha;
    zbeta = flux_est_beta;
    if(flux_est_alpha >= sat) zalpha = sat;
    if(flux_est_alpha <= -sat) zalpha = -sat;
    if(flux_est_beta >= sat) zbeta = sat;
    if(flux_est_beta <= -sat) zbeta = -sat;

    y2alpha = m3*y2alpha + m4*zalpha;
    y2beta  = m3*y2beta  + m4*zbeta;

    flux_est_alpha = (y1alpha + y2alpha);
    flux_est_beta  = (y1beta  + y2beta);
//    DacaRegs.DACVALS.all = (flux_est_alpha*2000.0/2.0) + 2000;
//    DacbRegs.DACVALS.all = (flux_est_beta*2000.0/2.0) + 2000;
    //DacaRegs.DACVALS.all = phir_beta_st * 1024.0 / 2.0 + 1024;
    //float pos = atan2(phir_alpha_st, phir_beta_st);
    //DacbRegs.DACVALS.all = pos * 1024.0 / 7 + 1024;

    // Modelo Adaptativo
    float dphir_alpha_rt = -(1/Tr)*phir_alpha_rt - wr*phir_beta_rt  + (Lm/Tr)*ialpha;
    float dphir_beta_rt  = -(1/Tr)*phir_beta_rt  + wr*phir_alpha_rt + (Lm/Tr)*ibeta;
    phir_alpha_rt  = (phir_alpha_rt + (Ts/2)*dphir_alpha_rt + (Ts/2)*dphia_k1);
    phir_beta_rt   = phir_beta_rt  + (Ts/2)*dphir_beta_rt  + (Ts/2)*dphib_k1;

    // Mecanismo de adaptação
//    float prodk = -(flux_est_beta*phir_alpha_rt - flux_est_alpha*phir_beta_rt);
    float prodk = (-(-flux_est_alpha*phir_alpha_rt) - (-flux_est_beta*phir_beta_rt));

    wr = (wrk1 + fpb_b*prodk1 + fpb_a*prodk);
//    wr = (DPI/60.0)*Velo;

//    DacaRegs.DACVALS.all = 1*(phir_alpha_rt * 2000.0 / 2.0) + 2000; // Amarelo
//    DacbRegs.DACVALS.all = 1*(-flux_est_beta * 2000.0 / 2.0) + 2000; // Azul

    // Filtro passa-baixas
    wrf = (wrf + Ts*10*wr)/(1+Ts*10);
    //DacbRegs.DACVALS.all = wrf * 1024.0 / 1000.0 + 1024;

    // Atualização das variáveis
//    xka1 = xka;
//    xkb1 = xkb;

    dphia_k1 = dphir_alpha_rt;
    dphib_k1 = dphir_beta_rt;

    prodk1 = prodk;
    //wr = Velo*DPI*p/60.0;
    wrk1 = wr;
    //return pos;
    return -1.5*(60.0/DPI)*(1.0/p)*wrf - 250.0;  // conversão de rad/s para rpm
}

#elif defined(EKF)


int init = 1;

// Iniciando xhat
float xh1 = 1, xh2 = 1, xh3 = 1, xh4 = 1, xh5 = 1, xt1 = 0, xt2 = 0, xt3 = 0, xt4 = 0, xt5 = 0;

// Iniciando Phat
float ph11 = 0, ph12 = 0, ph13 = 0, ph14 = 0, ph15 = 0;
float ph21 = 0, ph22 = 0, ph23 = 0, ph24 = 0, ph25 = 0;
float ph31 = 0, ph32 = 0, ph33 = 0, ph34 = 0, ph35 = 0;
float ph41 = 0, ph42 = 0, ph43 = 0, ph44 = 0, ph45 = 0;
float ph51 = 0, ph52 = 0, ph53 = 0, ph54 = 0, ph55 = 0;

// Iniciando Ptil
float pt11 = 0, pt12 = 0, pt13 = 0, pt14 = 0, pt15 = 0;
float pt21 = 0, pt22 = 0, pt23 = 0, pt24 = 0, pt25 = 0;
float pt31 = 0, pt32 = 0, pt33 = 0, pt34 = 0, pt35 = 0;
float pt41 = 0, pt42 = 0, pt43 = 0, pt44 = 0, pt45 = 0;
float pt51 = 0, pt52 = 0, pt53 = 0, pt54 = 0, pt55 = 0;

float ref_OBS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){ // Observador de fluxo rotórico e de velocidade filtro de Kalman estendido
    // Transformação de clarke de v
    // Mudanças: valpha e vbeta trocados e mecanismo de adaptação
    float valpha = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float vbeta = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    // Transformação de clarke de i
    float ialpha = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ibeta = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);
//
//    DacaRegs.DACVALS.all = (ialpha*2000.0/1.0) + 2000.0;
//    DacbRegs.DACVALS.all = (ibeta*2000.0/1.0) + 2000.0;
    // Iniciando gk
    float k11 = 0, k21 = 0, k31 = 0, k41 = 0, k51 = 0, k12 = 0, k22 = 0, k32 = 0, k42 = 0, k52 = 0;

    // Matriz A
    float a11 = (1 - Ts/Tlinha);                 //( = 0.998891379207670)
    float a13 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));      //( = 8.000435870123950e-04)
    float a14 = ((2*xh5*Ts*Lm)/(sigma*Ls*Lr));     //( = 9.552213001383126e-06*2*xh5)  xh5 entra em A
    float a22 = (1 - Ts/Tlinha);                 //( = 0.998891379207670)
    float a23 = ((-2*xh5*Ts*Lm)/(sigma*Ls*Lr));    //( = 9.552213001383126e-06*2*xh5)  xh5 entra em A
    float a24 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));      //( =  8.000435870123950e-04)
    float a31 = (Ts*Lm/Tr);                      //( = 2.467974457215837e-04)
    float a33 = (1 - Ts/Tr);                     //( = 0.999720817369093)
    float a34 = -Ts*2*xh5;                         //( = -3.333333333333333e-06*2*xh5) xh5 entra em A
    float a42 = (Ts*Lm/Tr);                      //( = 2.467974457215837e-04)
    float a43 =  Ts*2*xh5;                         //( = 3.333333333333333e-06*2*xh5)  xh5 entra em A
    float a44 = (1 - Ts/Tr);                     //( = 0.999720817369093)

    // Matriz B
    float b11 = (Ts/(sigma*Ls));
    float b22 = (Ts/(sigma*Ls));

    // Matriz Q
    float q1 = 1e-6;
    float q2 = q1, q3 = q1, q4 = q1, q5 = 0.5*q1*1e3;

    // Matriz R
    float r11 = 1, r22 = 1;

    //=========== EKF ==================

    // Predição
    // 1) variáveis de estado
    // xtil = A*xhat + B*u;

    // Traduzindo
    xt1 = b11*valpha + a11*xh1 + a13*xh3 + a14*xh4;
    xt2 = b22*vbeta + a22*xh2 + a23*xh3 + a24*xh4;
    xt3 = a31*xh1 + a33*xh3 + a34*xh4;
    xt4 = a42*xh2 + a43*xh3 + a44*xh4;
    xt5 = xh5;


    // Matriz G
    float g11 = (1 - Ts/Tlinha);                    //( = 0.998891379207670)

    float g13 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));         //( = 8.000435870123950e-04)
    float g14 = ((Ts*Lm)/(sigma*Ls*Lr)) * (2*xt5);  //( = 9.552213001383126e-06*2*xt5)  xt5 entra na matriz G (velocidade)
    float g15 = ((Ts*Lm)/(sigma*Ls*Lr)) * (xt4);    //( = 9.552213001383126e-06*xt4)    xt4 entra na matriz G (phi_beta)

    float g22 = (1-Ts/Tlinha);                      //( = 0.998891379207670)
    float g23 = ((Ts*Lm)/(sigma*Ls*Lr)) * (-2*xt5); //( = 9.552213001383126e-06*-2*xt5) xt5 entra na matriz G (velocidade)
    float g24 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));         //( = 8.000435870123950e-04)
    float g25 = ((-Ts*Lm)/(sigma*Ls*Lr)) * (xt3);   //( = -9.552213001383126e-06*xt3) xt3 entra na matriz G (phi_alpha)
    float g31 = (Ts*Lm/Tr);                         //( = 2.467974457215837e-04)

    float g33 =  (1 - Ts/Tr);                       //( = 0.999720817369093)
    float g34 = -Ts * (2*xt5);                      //( = -3.333333333333333e-06*2*xt5) xt5 entra na matriz G (velocidade)
    float g35 = -Ts * (xt4);                        //( = -3.333333333333333e-06*xt4) xt4 entra na matriz G (phi_beta)


    float g42 = Ts*Lm/Tr;                           //( = 2.467974457215837e-04)
    float g43 = Ts * (2*xt5);                       //( = 3.333333333333333e-06*2*xt5) xt5 entra na matriz G (velocidade)
    float g44 = (1 - Ts/Tr);                        //( = 0.999720817369093)
    float g45 = Ts * (xt3);                         //( = 3.333333333333333e-06*xt3) xt3 entra na matriz G (phi_alpha)



    // 2) covariância do erro
    // Ptil = G*Phat*G'+ Q;

    // Traduzindo
    pt11 = q1 + g11*(g11*ph11 + g13*ph31 + g14*ph41 + g15*ph51) + g13*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g14*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g15*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt12 = g22*(g11*ph12 + g13*ph32 + g14*ph42 + g15*ph52) + g23*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g24*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g25*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt13 = g31*(g11*ph11 + g13*ph31 + g14*ph41 + g15*ph51) + g33*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g34*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g35*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt14 = g42*(g11*ph12 + g13*ph32 + g14*ph42 + g15*ph52) + g43*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g44*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g45*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt15 = g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55;

    pt21 = g11*(g22*ph21 + g23*ph31 + g24*ph41 + g25*ph51) + g13*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g14*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g15*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt22 = q2 + g22*(g22*ph22 + g23*ph32 + g24*ph42 + g25*ph52) + g23*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g24*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g25*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt23 = g31*(g22*ph21 + g23*ph31 + g24*ph41 + g25*ph51) + g33*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g34*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g35*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt24 = g42*(g22*ph22 + g23*ph32 + g24*ph42 + g25*ph52) + g43*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g44*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g45*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt25 = g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55;

    pt31 = g11*(g31*ph11 + g33*ph31 + g34*ph41 + g35*ph51) + g13*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g14*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g15*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt32 = g22*(g31*ph12 + g33*ph32 + g34*ph42 + g35*ph52) + g23*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g24*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g25*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt33 = q3 + g31*(g31*ph11 + g33*ph31 + g34*ph41 + g35*ph51) + g33*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g34*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g35*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt34 = g42*(g31*ph12 + g33*ph32 + g34*ph42 + g35*ph52) + g43*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g44*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g45*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt35 = g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55;

    pt41 = g11*(g42*ph21 + g43*ph31 + g44*ph41 + g45*ph51) + g13*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g14*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g15*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt42 = g22*(g42*ph22 + g43*ph32 + g44*ph42 + g45*ph52) + g23*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g24*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g25*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt43 = g31*(g42*ph21 + g43*ph31 + g44*ph41 + g45*ph51) + g33*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g34*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g35*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt44 = q4 + g42*(g42*ph22 + g43*ph32 + g44*ph42 + g45*ph52) + g43*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g44*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g45*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt45 = g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55;

    pt51 = g11*ph51 + g13*ph53 + g14*ph54 + g15*ph55;
    pt52 = g22*ph52 + g23*ph53 + g24*ph54 + g25*ph55;
    pt53 = g31*ph51 + g33*ph53 + g34*ph54 + g35*ph55;
    pt54 = g42*ph52 + g43*ph53 + g44*ph54 + g45*ph55;
    pt55 = ph55 + q5;



    // Etapa de atualização
    // 3) ganho de Kalman
    // kk = Ptil*C'*inv(C*Ptil*C'+ R);

    // Traduzindo
    float det = (pt11*pt22 - pt12*pt21 + pt11*r22 + pt22*r11 + r11*r22);

    k11 = (1/det)*(pt11*(pt22 + r22)) - (pt12*pt21);
    k12 = (1/det)*(pt12*(pt11 + r11)) - (pt11*pt12);

    k21 = (1/det)*(pt21*(pt22 + r22)) - (pt21*pt22);
    k22 = (1/det)*(pt22*(pt11 + r11)) - (pt12*pt21);

    k31 = (1/det)*(pt31*(pt22 + r22)) - (pt21*pt32);
    k32 = (1/det)*(pt32*(pt11 + r11)) - (pt12*pt31);

    k41 = (1/det)*(pt41*(pt22 + r22)) - (pt21*pt42);
    k42 = (1/det)*(pt42*(pt11 + r11)) - (pt12*pt41);

    k51 = (1/det)*(pt51*(pt22 + r22)) - (pt21*pt52);
    k52 = (1/det)*(pt52*(pt11 + r11)) - (pt12*pt51);


    // 4) Atualização dos estados preditos
    // xhat = xtil + kk*(y - C*xtil);

    // Traduzindo
    xh1 = xt1 - k11*(xt1 - ialpha) - k12*(xt2 - ibeta);
    xh2 = xt2 - k21*(xt1 - ialpha) - k22*(xt2 - ibeta);
    xh3 = xt3 - k31*(xt1 - ialpha) - k32*(xt2 - ibeta);
    xh4 = xt4 - k41*(xt1 - ialpha) - k42*(xt2 - ibeta);
    xh5 = xt5 - k51*(xt1 - ialpha) - k52*(xt2 - ibeta);

//    DacaRegs.DACVALS.all = (xh3*2000.0/1.0) + 2000.0;
//    DacbRegs.DACVALS.all = (xh4*2000.0/1.0) + 2000.0;

    // 5) Atualização da covariância do erro
    // Phat = (eye(5) - kk*C)*Ptil;

    // Traduzindo
    ph11 = - k12*pt21 - pt11*(k11 - 1);
    ph12 = - k12*pt22 - pt12*(k11 - 1);
    ph13 = - k12*pt23 - pt13*(k11 - 1);
    ph14 = - k12*pt24 - pt14*(k11 - 1);
    ph15 = - k12*pt25 - pt15*(k11 - 1);

    ph21 = - k21*pt11 - pt21*(k22 - 1);
    ph22 = - k21*pt12 - pt22*(k22 - 1);
    ph23 = - k21*pt13 - pt23*(k22 - 1);
    ph24 = - k21*pt14 - pt24*(k22 - 1);
    ph25 = - k21*pt15 - pt25*(k22 - 1);

    ph31 = pt31 - k31*pt11 - k32*pt21;
    ph32 = pt32 - k31*pt12 - k32*pt22;
    ph33 = pt33 - k31*pt13 - k32*pt23;
    ph34 = pt34 - k31*pt14 - k32*pt24;
    ph35 = pt35 - k31*pt15 - k32*pt25;

    ph41 = pt41 - k41*pt11 - k42*pt21;
    ph42 = pt42 - k41*pt12 - k42*pt22;
    ph43 = pt43 - k41*pt13 - k42*pt23;
    ph44 = pt44 - k41*pt14 - k42*pt24;
    ph45 = pt45 - k41*pt15 - k42*pt25;

    ph51 = pt51 - k51*pt11 - k52*pt21;
    ph52 = pt52 - k51*pt12 - k52*pt22;
    ph53 = pt53 - k51*pt13 - k52*pt23;
    ph54 = pt54 - k51*pt14 - k52*pt24;
    ph55 = pt55 - k51*pt15 - k52*pt25;

    return -6.0*xh5;
}

#elif defined(EKF_MRAS)


int init = 1;

// Iniciando xhat
float xh1 = 1, xh2 = 1, xh3 = 1, xh4 = 1, xh5 = 1, xt1 = 0, xt2 = 0, xt3 = 0, xt4 = 0, xt5 = 0;

// Iniciando Phat
float ph11 = 0, ph12 = 0, ph13 = 0, ph14 = 0, ph15 = 0;
float ph21 = 0, ph22 = 0, ph23 = 0, ph24 = 0, ph25 = 0;
float ph31 = 0, ph32 = 0, ph33 = 0, ph34 = 0, ph35 = 0;
float ph41 = 0, ph42 = 0, ph43 = 0, ph44 = 0, ph45 = 0;
float ph51 = 0, ph52 = 0, ph53 = 0, ph54 = 0, ph55 = 0;

// Iniciando Ptil
float pt11 = 0, pt12 = 0, pt13 = 0, pt14 = 0, pt15 = 0;
float pt21 = 0, pt22 = 0, pt23 = 0, pt24 = 0, pt25 = 0;
float pt31 = 0, pt32 = 0, pt33 = 0, pt34 = 0, pt35 = 0;
float pt41 = 0, pt42 = 0, pt43 = 0, pt44 = 0, pt45 = 0;
float pt51 = 0, pt52 = 0, pt53 = 0, pt54 = 0, pt55 = 0;

float aux_alpha = 0, aux_beta = 0, aux_alpha2 = 0, aux_alpha3 = 0, aux_beta2 = 0, aux_beta3 = 0;

float ref_OBS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){ // Observador de fluxo rotórico e de velocidade filtro de Kalman estendido
    // Transformação de clarke de v
    // Mudanças: valpha e vbeta trocados e mecanismo de adaptação
    float valpha = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float vbeta = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    // Transformação de clarke de i
    float ialpha = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ibeta = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);
    data2->ia = ialpha;
    data2->ib = ibeta;
//    DacaRegs.DACVALS.all = 1*(ialpha * 2000.0 / 1.0) + 2000; // Amarelo
//    DacbRegs.DACVALS.all = 1*(ibeta * 2000.0 / 1.0) + 2000; // Azul

    float xka = (valpha - Rs*ialpha);
    float xkb = (vbeta - Rs*ibeta);

    // Integração pura
    //    aux_alpha = aux_alpha + (Ts/2)*xka + (Ts/2)*xka1;
    //    aux_beta  = aux_beta  + (Ts/2)*xkb + (Ts/2)*xkb1;
    //
    //    float phir_alpha_st = (Lr/Lm)*(aux_alpha - sigma*Ls*ialpha );
    //    float phir_beta_st  = (Lr/Lm)*(aux_beta  - sigma*Ls*ibeta );

    // Integração do fluxo com correção
//    float g1 = 9.993602e-01, g2 = 3.198976e-06, g3 = 9.993602e-01, g4 = 6.397952e-04;
//    float g1 = 9.987208e-01, g2 = 1.279181e-05, g3 = 9.987208e-01, g4 = 1.279181e-03; // wc = 100
//    float g1 = 9.993602e-01, g2 = 1.279590e-05, g3 = 9.993602e-01, g4 = 6.397952e-04; // wc = 50
//    float g1 = 9.998720e-01, g2 = 1.279918e-05, g3 = 9.998720e-01, g4 = 1.279918e-04; // wc = 10
//    float g1 = 9.936204e-01, g2 = 1.275913e-05, g3 = 9.936204e-01, g4 = 6.379564e-03; // wc = 500
    float g1 = 9.872816e-01, g2 = 1.271843e-05, g3 = 9.872816e-01, g4 = 1.271843e-02; // wc = 1000


    aux_alpha = g1*aux_alpha + g2*xka;
    aux_beta  = g1*aux_beta  + g2*xkb;

    if(aux_alpha3 >= 0.5) aux_alpha3 = 0.5;
    if(aux_alpha3 <= -0.5) aux_alpha3 = -0.5;
    if(aux_beta3 >= 0.5) aux_beta3 = 0.5;
    if(aux_beta3 <= -0.5) aux_beta3 = -0.5;

    aux_alpha2 = g3*aux_alpha2 + g4*aux_alpha3;
    aux_beta2  = g3*aux_beta2  + g4*aux_beta3;

    aux_alpha3 = aux_alpha + aux_alpha2;
    aux_beta3  = aux_beta  + aux_beta2;


    float phia_mras = (Lr/Lm)*(aux_alpha3 - sigma*Ls*ialpha );
    float phib_mras  = (Lr/Lm)*(aux_beta3  - sigma*Ls*ibeta );

    data2->phia = phia_mras;
    data2->phib = phib_mras;

//    DacaRegs.DACVALS.all = (phia_mras * 1024) + 1024;
//    DacbRegs.DACVALS.all = (phib_mras * 1024) + 1024;

    // Matriz A
    float a11 = (1 - Ts/Tlinha);                 //( = 0.998891379207670)
    float a13 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));      //( = 8.000435870123950e-04)
    float a14 = ((2*xh5*Ts*Lm)/(sigma*Ls*Lr));     //( = 9.552213001383126e-06*2*xh5)  xh5 entra em A
    float a22 = (1 - Ts/Tlinha);                 //( = 0.998891379207670)
    float a23 = ((-2*xh5*Ts*Lm)/(sigma*Ls*Lr));    //( = 9.552213001383126e-06*2*xh5)  xh5 entra em A
    float a24 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));      //( =  8.000435870123950e-04)
    float a31 = (Ts*Lm/Tr);                      //( = 2.467974457215837e-04)
    float a33 = (1 - Ts/Tr);                     //( = 0.999720817369093)
    float a34 = -Ts*2*xh5;                         //( = -3.333333333333333e-06*2*xh5) xh5 entra em A
    float a42 = (Ts*Lm/Tr);                      //( = 2.467974457215837e-04)
    float a43 =  Ts*2*xh5;                         //( = 3.333333333333333e-06*2*xh5)  xh5 entra em A
    float a44 = (1 - Ts/Tr);                     //( = 0.999720817369093)

    // Matriz B
    float b11 = (Ts/(sigma*Ls));

    // Matriz Q
    float q1 = 1e-6;
    float q2 = q1, q3 = q1, q4 = q1, q5 = q1*1e3;

    // Matriz R
    float r11 = 0.1;

    //=========== EKF ==================

    // Predição
    // 1) variáveis de estado
    // xtil = A*xhat + B*u;

    // Traduzindo
    xt1 = b11*valpha + a11*xh1 + a13*xh3 + a14*xh4;
    xt2 = b11*vbeta + a22*xh2 + a23*xh3 + a24*xh4;
    xt3 = a31*xh1 + a33*xh3 + a34*xh4;
    xt4 = a42*xh2 + a43*xh3 + a44*xh4;
    xt5 = xh5;


    // Matriz G
    float g11 = (1 - Ts/Tlinha);                    //( = 0.998891379207670)

    float g13 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));         //( = 8.000435870123950e-04)
    float g14 = ((Ts*Lm)/(sigma*Ls*Lr)) * (2*xt5);  //( = 9.552213001383126e-06*2*xt5)  xt5 entra na matriz G (velocidade)
    float g15 = ((Ts*Lm)/(sigma*Ls*Lr)) * (xt4);    //( = 9.552213001383126e-06*xt4)    xt4 entra na matriz G (phi_beta)

    float g22 = (1-Ts/Tlinha);                      //( = 0.998891379207670)
    float g23 = ((Ts*Lm)/(sigma*Ls*Lr)) * (-2*xt5); //( = 9.552213001383126e-06*-2*xt5) xt5 entra na matriz G (velocidade)
    float g24 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));         //( = 8.000435870123950e-04)
    float g25 = ((-Ts*Lm)/(sigma*Ls*Lr)) * (xt3);   //( = -9.552213001383126e-06*xt3) xt3 entra na matriz G (phi_alpha)
    float g31 = (Ts*Lm/Tr);                         //( = 2.467974457215837e-04)

    float g33 =  (1 - Ts/Tr);                       //( = 0.999720817369093)
    float g34 = -Ts * (2*xt5);                      //( = -3.333333333333333e-06*2*xt5) xt5 entra na matriz G (velocidade)
    float g35 = -Ts * (xt4);                        //( = -3.333333333333333e-06*xt4) xt4 entra na matriz G (phi_beta)


    float g42 = Ts*Lm/Tr;                           //( = 2.467974457215837e-04)
    float g43 = Ts * (2*xt5);                       //( = 3.333333333333333e-06*2*xt5) xt5 entra na matriz G (velocidade)
    float g44 = (1 - Ts/Tr);                        //( = 0.999720817369093)
    float g45 = Ts * (xt3);                         //( = 3.333333333333333e-06*xt3) xt3 entra na matriz G (phi_alpha)



    // 2) covariância do erro
    // Ptil = G*Phat*G'+ Q;

    // Traduzindo
    pt11 = q1 + g11*(g11*ph11 + g13*ph31 + g14*ph41 + g15*ph51) + g13*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g14*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g15*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt12 = g22*(g11*ph12 + g13*ph32 + g14*ph42 + g15*ph52) + g23*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g24*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g25*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt13 = g31*(g11*ph11 + g13*ph31 + g14*ph41 + g15*ph51) + g33*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g34*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g35*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt14 = g42*(g11*ph12 + g13*ph32 + g14*ph42 + g15*ph52) + g43*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g44*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g45*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt15 = g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55;

    pt21 = g11*(g22*ph21 + g23*ph31 + g24*ph41 + g25*ph51) + g13*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g14*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g15*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt22 = q2 + g22*(g22*ph22 + g23*ph32 + g24*ph42 + g25*ph52) + g23*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g24*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g25*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt23 = g31*(g22*ph21 + g23*ph31 + g24*ph41 + g25*ph51) + g33*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g34*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g35*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt24 = g42*(g22*ph22 + g23*ph32 + g24*ph42 + g25*ph52) + g43*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g44*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g45*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt25 = g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55;

    pt31 = g11*(g31*ph11 + g33*ph31 + g34*ph41 + g35*ph51) + g13*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g14*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g15*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt32 = g22*(g31*ph12 + g33*ph32 + g34*ph42 + g35*ph52) + g23*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g24*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g25*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt33 = q3 + g31*(g31*ph11 + g33*ph31 + g34*ph41 + g35*ph51) + g33*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g34*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g35*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt34 = g42*(g31*ph12 + g33*ph32 + g34*ph42 + g35*ph52) + g43*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g44*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g45*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt35 = g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55;

    pt41 = g11*(g42*ph21 + g43*ph31 + g44*ph41 + g45*ph51) + g13*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g14*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g15*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt42 = g22*(g42*ph22 + g43*ph32 + g44*ph42 + g45*ph52) + g23*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g24*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g25*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt43 = g31*(g42*ph21 + g43*ph31 + g44*ph41 + g45*ph51) + g33*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g34*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g35*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt44 = q4 + g42*(g42*ph22 + g43*ph32 + g44*ph42 + g45*ph52) + g43*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g44*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g45*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt45 = g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55;

    pt51 = g11*ph51 + g13*ph53 + g14*ph54 + g15*ph55;
    pt52 = g22*ph52 + g23*ph53 + g24*ph54 + g25*ph55;
    pt53 = g31*ph51 + g33*ph53 + g34*ph54 + g35*ph55;
    pt54 = g42*ph52 + g43*ph53 + g44*ph54 + g45*ph55;
    pt55 = ph55 + q5;

    // Cálculo de resíduo
    float res = phib_mras*xt3 - phia_mras*xt4;
//    float res = phib_mras*xt4 - phia_mras*xt3;


    // 4) Atualização dos estados preditos
    // xhat = xtil + kk*(res);
    xh1 = xt1 + (pt15*res)/(pt55 + r11);
    xh2 = xt2 + (pt25*res)/(pt55 + r11);
    xh3 = xt3 + (pt35*res)/(pt55 + r11);
    xh4 = xt4 + (pt45*res)/(pt55 + r11);
    xh5 = xt5 + (pt55*res)/(pt55 + r11);
//    DacaRegs.DACVALS.all = (phib_mras * 2000) + 2000;
//    DacbRegs.DACVALS.all = (xh4 * 2000) + 2000;

    ph11 = pt11 - (pt15*pt51)/(pt55 + r11);
    ph12 = pt12 - (pt15*pt52)/(pt55 + r11);
    ph13 = pt13 - (pt15*pt53)/(pt55 + r11);
    ph14 = pt14 - (pt15*pt54)/(pt55 + r11);
    ph15 = pt15 - (pt15*pt55)/(pt55 + r11);

    ph21 = pt21 - (pt25*pt51)/(pt55 + r11);
    ph22 = pt22 - (pt25*pt52)/(pt55 + r11);
    ph23 = pt23 - (pt25*pt53)/(pt55 + r11);
    ph24 = pt24 - (pt25*pt54)/(pt55 + r11);
    ph25 = pt25 - (pt25*pt55)/(pt55 + r11);

    ph31 = pt31 - (pt35*pt51)/(pt55 + r11);
    ph32 = pt32 - (pt35*pt52)/(pt55 + r11);
    ph33 = pt33 - (pt35*pt53)/(pt55 + r11);
    ph34 = pt34 - (pt35*pt54)/(pt55 + r11);
    ph35 = pt35 - (pt35*pt55)/(pt55 + r11);

    ph41 = pt41 - (pt45*pt51)/(pt55 + r11);
    ph42 = pt42 - (pt45*pt52)/(pt55 + r11);
    ph43 = pt43 - (pt45*pt53)/(pt55 + r11);
    ph44 = pt44 - (pt45*pt54)/(pt55 + r11);
    ph45 = pt45 - (pt45*pt55)/(pt55 + r11);

    ph51 = -pt51*(pt55/(pt55 + r11) - 1);
    ph52 = -pt52*(pt55/(pt55 + r11) - 1);
    ph53 = -pt53*(pt55/(pt55 + r11) - 1);
    ph54 = -pt54*(pt55/(pt55 + r11) - 1);
    ph55 = -pt55*(pt55/(pt55 + r11) - 1);
    return -xh5*4.45 + 375;
}

#elif defined(EKF_SMO)

int init = 1;

// Iniciando xhat
float xh1 = 1, xh2 = 1, xh3 = 1, xh4 = 1, xh5 = 1, xt1 = 0, xt2 = 0, xt3 = 0, xt4 = 0, xt5 = 0;

// Iniciando Phat
float ph11 = 0, ph12 = 0, ph13 = 0, ph14 = 0, ph15 = 0;
float ph21 = 0, ph22 = 0, ph23 = 0, ph24 = 0, ph25 = 0;
float ph31 = 0, ph32 = 0, ph33 = 0, ph34 = 0, ph35 = 0;
float ph41 = 0, ph42 = 0, ph43 = 0, ph44 = 0, ph45 = 0;
float ph51 = 0, ph52 = 0, ph53 = 0, ph54 = 0, ph55 = 0;

// Iniciando Ptil
float pt11 = 0, pt12 = 0, pt13 = 0, pt14 = 0, pt15 = 0;
float pt21 = 0, pt22 = 0, pt23 = 0, pt24 = 0, pt25 = 0;
float pt31 = 0, pt32 = 0, pt33 = 0, pt34 = 0, pt35 = 0;
float pt41 = 0, pt42 = 0, pt43 = 0, pt44 = 0, pt45 = 0;
float pt51 = 0, pt52 = 0, pt53 = 0, pt54 = 0, pt55 = 0;

float dialpha_est = 0, dibeta_est = 0, ialpha_est = 0, ibeta_est = 0, qalpha = 0, qbeta = 0, salpha = 1, sbeta = 1, salphak1 = 0, sbetak1 = 0, dflux_est_alpha = 0, dflux_est_beta= 0, flux_est_alpha= 0, flux_est_beta= 0, dqalpha = 0, dqbeta = 0;

float y1alpha = 0, y1beta = 0, y2alpha = 0, y2beta = 0, zalpha = 0, zbeta = 0;  // variáveis para integração das correntes

float ref_OBS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){ // Observador de fluxo rotórico e de velocidade filtro de Kalman estendido
    // Transformação de clarke de v
    // Mudanças: valpha e vbeta trocados e mecanismo de adaptação
    float vbeta = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float valpha = -(2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    // Transformação de clarke de i
    float ialpha = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ibeta = -(2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    dialpha_est = qalpha - k1*ialpha_est + k2*valpha;
    dibeta_est  = qbeta  - k1*ibeta_est  + k2*vbeta;

    // Integração da corrente
    ialpha_est = ialpha_est + Ts*(dialpha_est);
    ibeta_est  = ibeta_est  + Ts*(dibeta_est);

//    DacaRegs.DACVALS.all = 1*(ialpha* 2000.0 / 1.0) + 2000; // Amarelo
//    DacbRegs.DACVALS.all = 1*(ibeta * 2000.0 / 1.0) + 2000; // Azul

    // Cálculo das funções "s" em alpha/beta
    //salpha = ialpha_est - ialpha;
    //sbeta  = ibeta_est  - ibeta;

    salpha = (ialpha_est - ialpha);
    sbeta  = (ibeta_est  - ibeta);

    //DacaRegs.DACVALS.all = 1*(qalpha * 2048.0 / 10000.0) + 1024; // Amarelo
    //DacbRegs.DACVALS.all = 1*(qbeta * 2048.0 / 10000.0) + 1024; // Azul
    //int_sa += Ts*(salpha-salphak1);

    // Cálculo de dq
//    dqalpha = - (1/Ts)*( (1+Ts*gamma1)*salpha - salphak1);
//    dqbeta  = - (1/Ts)*( (1+Ts*gamma2)*sbeta - sbetak1 );

    //dqalpha = ((1+Ts*gamma1)*salpha - salphak1);
    //dqbeta =  (sbeta - sbetak1);

    // Cálculo de q
    qalpha = qalpha - (1/Ts)*( (1+Ts*gamma1)*salpha - salphak1);
    qbeta  = qbeta  - (1/Ts)*( (1+Ts*gamma2)*sbeta - sbetak1 );

//    qalpha += dqalpha;
//    qbeta += dqbeta;


//    DacaRegs.DACVALS.all = 1*(qalpha * 1.0 / 200.0) + 1024; // Amarelo
//    DacbRegs.DACVALS.all = 1*(qbeta * 1.0 / 200.0) + 1024; // Azul


    // Cálculo das derivadas dos fluxos
    dflux_est_alpha = - qalpha/gamma;
    dflux_est_beta  = - qbeta/gamma;


    // Integração dos fluxos com correção
    //float m1 = 9.996801e-01, m2 = 3.199488e-06, m3 = 9.996801e-01, m4 = 3.199488e-04, sat =0.1; // wc = 100
//    float m1 = 9.996001e-01, m2 = 3.199360e-06, m3 = 9.996001e-01, m4 = 3.999200e-04, sat = 0.2; // wc = 125
    //float m1 = 9.841273e-01, m2 = 1.587268e-05, m3 = 9.841273e-01, m4 = 1.587268e-02, sat = 0.2; // wc = 1000
    float m1 = 9.984013e-01, m2 = 1.598721e-05, m3 = 9.984013e-01, m4 = 1.598721e-03, sat = 0.1; // wc = 10

    y1alpha = m1*y1alpha +  m2*dflux_est_alpha;
    y1beta  = m1*y1beta  + m2*dflux_est_beta;
    zalpha = flux_est_alpha;
    zbeta = flux_est_beta;
    if(flux_est_alpha >= sat) zalpha = sat;
    if(flux_est_alpha <= -sat) zalpha = -sat;
    if(flux_est_beta >= sat) zbeta = sat;
    if(flux_est_beta <= -sat) zbeta = -sat;

    y2alpha = m3*y2alpha + m4*zalpha;
    y2beta  = m3*y2beta  + m4*zbeta;

    flux_est_alpha = -(y1alpha + y2alpha);
    flux_est_beta  = (y1beta  + y2beta);
//    flux_est_alpha += Ts*dflux_est_alpha;
//    flux_est_beta += Ts*dflux_est_beta;


    // Matriz A
    float a11 = (1 - Ts/Tlinha);                 //( = 0.998891379207670)
    float a13 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));      //( = 8.000435870123950e-04)
    float a14 = ((2*xh5*Ts*Lm)/(sigma*Ls*Lr));     //( = 9.552213001383126e-06*2*xh5)  xh5 entra em A
    float a22 = (1 - Ts/Tlinha);                 //( = 0.998891379207670)
    float a23 = ((-2*xh5*Ts*Lm)/(sigma*Ls*Lr));    //( = 9.552213001383126e-06*2*xh5)  xh5 entra em A
    float a24 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));      //( =  8.000435870123950e-04)
    float a31 = (Ts*Lm/Tr);                      //( = 2.467974457215837e-04)
    float a33 = (1 - Ts/Tr);                     //( = 0.999720817369093)
    float a34 = -Ts*2*xh5;                         //( = -3.333333333333333e-06*2*xh5) xh5 entra em A
    float a42 = (Ts*Lm/Tr);                      //( = 2.467974457215837e-04)
    float a43 =  Ts*2*xh5;                         //( = 3.333333333333333e-06*2*xh5)  xh5 entra em A
    float a44 = (1 - Ts/Tr);                     //( = 0.999720817369093)

    // Matriz B
    float b11 = (Ts/(sigma*Ls));

    // Matriz Q
    float q1 = 1e-6;
    float q2 = q1, q3 = q1, q4 = q1, q5 = 0.5*q1*1e3;

    // Matriz R
//    float r11 = 0.2;
    float r11 = 0.1;
    //=========== EKF ==================

    // Predição
    // 1) variáveis de estado
    // xtil = A*xhat + B*u;

    // Traduzindo
//    xt1 = b11*vbeta + a11*xh1 + a13*xh3 + a14*xh4;
//    xt2 = -b11*valpha + a22*xh2 + a23*xh3 + a24*xh4;
    xt1 = b11*valpha + a11*xh1 + a13*xh3 + a14*xh4;
    xt2 = -b11*vbeta + a22*xh2 + a23*xh3 + a24*xh4;
    xt3 = a31*xh1 + a33*xh3 + a34*xh4;
    xt4 = a42*xh2 + a43*xh3 + a44*xh4;
    xt5 = xh5;


    // Matriz G
    float g11 = (1 - Ts/Tlinha);                    //( = 0.998891379207670)

    float g13 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));         //( = 8.000435870123950e-04)
    float g14 = ((Ts*Lm)/(sigma*Ls*Lr)) * (2*xt5);  //( = 9.552213001383126e-06*2*xt5)  xt5 entra na matriz G (velocidade)
    float g15 = ((Ts*Lm)/(sigma*Ls*Lr)) * (xt4);    //( = 9.552213001383126e-06*xt4)    xt4 entra na matriz G (phi_beta)

    float g22 = (1-Ts/Tlinha);                      //( = 0.998891379207670)
    float g23 = ((Ts*Lm)/(sigma*Ls*Lr)) * (-2*xt5); //( = 9.552213001383126e-06*-2*xt5) xt5 entra na matriz G (velocidade)
    float g24 = ((Ts*Lm)/(sigma*Ls*Lr*Tr));         //( = 8.000435870123950e-04)
    float g25 = ((-Ts*Lm)/(sigma*Ls*Lr)) * (xt3);   //( = -9.552213001383126e-06*xt3) xt3 entra na matriz G (phi_alpha)
    float g31 = (Ts*Lm/Tr);                         //( = 2.467974457215837e-04)

    float g33 =  (1 - Ts/Tr);                       //( = 0.999720817369093)
    float g34 = -Ts * (2*xt5);                      //( = -3.333333333333333e-06*2*xt5) xt5 entra na matriz G (velocidade)
    float g35 = -Ts * (xt4);                        //( = -3.333333333333333e-06*xt4) xt4 entra na matriz G (phi_beta)


    float g42 = Ts*Lm/Tr;                           //( = 2.467974457215837e-04)
    float g43 = Ts * (2*xt5);                       //( = 3.333333333333333e-06*2*xt5) xt5 entra na matriz G (velocidade)
    float g44 = (1 - Ts/Tr);                        //( = 0.999720817369093)
    float g45 = Ts * (xt3);                         //( = 3.333333333333333e-06*xt3) xt3 entra na matriz G (phi_alpha)



    // 2) covariância do erro
    // Ptil = G*Phat*G'+ Q;

    // Traduzindo
    pt11 = q1 + g11*(g11*ph11 + g13*ph31 + g14*ph41 + g15*ph51) + g13*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g14*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g15*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt12 = g22*(g11*ph12 + g13*ph32 + g14*ph42 + g15*ph52) + g23*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g24*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g25*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt13 = g31*(g11*ph11 + g13*ph31 + g14*ph41 + g15*ph51) + g33*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g34*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g35*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt14 = g42*(g11*ph12 + g13*ph32 + g14*ph42 + g15*ph52) + g43*(g11*ph13 + g13*ph33 + g14*ph43 + g15*ph53) + g44*(g11*ph14 + g13*ph34 + g14*ph44 + g15*ph54) + g45*(g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55);
    pt15 = g11*ph15 + g13*ph35 + g14*ph45 + g15*ph55;

    pt21 = g11*(g22*ph21 + g23*ph31 + g24*ph41 + g25*ph51) + g13*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g14*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g15*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt22 = q2 + g22*(g22*ph22 + g23*ph32 + g24*ph42 + g25*ph52) + g23*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g24*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g25*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt23 = g31*(g22*ph21 + g23*ph31 + g24*ph41 + g25*ph51) + g33*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g34*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g35*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt24 = g42*(g22*ph22 + g23*ph32 + g24*ph42 + g25*ph52) + g43*(g22*ph23 + g23*ph33 + g24*ph43 + g25*ph53) + g44*(g22*ph24 + g23*ph34 + g24*ph44 + g25*ph54) + g45*(g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55);
    pt25 = g22*ph25 + g23*ph35 + g24*ph45 + g25*ph55;

    pt31 = g11*(g31*ph11 + g33*ph31 + g34*ph41 + g35*ph51) + g13*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g14*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g15*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt32 = g22*(g31*ph12 + g33*ph32 + g34*ph42 + g35*ph52) + g23*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g24*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g25*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt33 = q3 + g31*(g31*ph11 + g33*ph31 + g34*ph41 + g35*ph51) + g33*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g34*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g35*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt34 = g42*(g31*ph12 + g33*ph32 + g34*ph42 + g35*ph52) + g43*(g31*ph13 + g33*ph33 + g34*ph43 + g35*ph53) + g44*(g31*ph14 + g33*ph34 + g34*ph44 + g35*ph54) + g45*(g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55);
    pt35 = g31*ph15 + g33*ph35 + g34*ph45 + g35*ph55;

    pt41 = g11*(g42*ph21 + g43*ph31 + g44*ph41 + g45*ph51) + g13*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g14*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g15*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt42 = g22*(g42*ph22 + g43*ph32 + g44*ph42 + g45*ph52) + g23*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g24*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g25*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt43 = g31*(g42*ph21 + g43*ph31 + g44*ph41 + g45*ph51) + g33*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g34*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g35*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt44 = q4 + g42*(g42*ph22 + g43*ph32 + g44*ph42 + g45*ph52) + g43*(g42*ph23 + g43*ph33 + g44*ph43 + g45*ph53) + g44*(g42*ph24 + g43*ph34 + g44*ph44 + g45*ph54) + g45*(g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55);
    pt45 = g42*ph25 + g43*ph35 + g44*ph45 + g45*ph55;

    pt51 = g11*ph51 + g13*ph53 + g14*ph54 + g15*ph55;
    pt52 = g22*ph52 + g23*ph53 + g24*ph54 + g25*ph55;
    pt53 = g31*ph51 + g33*ph53 + g34*ph54 + g35*ph55;
    pt54 = g42*ph52 + g43*ph53 + g44*ph54 + g45*ph55;
    pt55 = ph55 + q5;

    // Cálculo de resíduo
//    float res = - flux_est_beta*(xt3) + flux_est_alpha*(xt4);
    float res = - flux_est_beta*(xt3) + flux_est_alpha*(xt4);
//    float res = phib_mras*xt4 - phia_mras*xt3;


    // 4) Atualização dos estados preditos
    // xhat = xtil + kk*(res);
    xh1 = xt1 + (pt15*res)/(pt55 + r11);
    xh2 = xt2 + (pt25*res)/(pt55 + r11);
    xh3 = xt3 + (pt35*res)/(pt55 + r11);
    xh4 = xt4 + (pt45*res)/(pt55 + r11);
    xh5 = xt5 + (pt55*res)/(pt55 + r11);

//    DacaRegs.DACVALS.all = (xh3 * 2000/1.0)  + 2000;
//    DacbRegs.DACVALS.all = (xh4  * 2000/1.0) + 2000;

    ph11 = pt11 - (pt15*pt51)/(pt55 + r11);
    ph12 = pt12 - (pt15*pt52)/(pt55 + r11);
    ph13 = pt13 - (pt15*pt53)/(pt55 + r11);
    ph14 = pt14 - (pt15*pt54)/(pt55 + r11);
    ph15 = pt15 - (pt15*pt55)/(pt55 + r11);

    ph21 = pt21 - (pt25*pt51)/(pt55 + r11);
    ph22 = pt22 - (pt25*pt52)/(pt55 + r11);
    ph23 = pt23 - (pt25*pt53)/(pt55 + r11);
    ph24 = pt24 - (pt25*pt54)/(pt55 + r11);
    ph25 = pt25 - (pt25*pt55)/(pt55 + r11);

    ph31 = pt31 - (pt35*pt51)/(pt55 + r11);
    ph32 = pt32 - (pt35*pt52)/(pt55 + r11);
    ph33 = pt33 - (pt35*pt53)/(pt55 + r11);
    ph34 = pt34 - (pt35*pt54)/(pt55 + r11);
    ph35 = pt35 - (pt35*pt55)/(pt55 + r11);

    ph41 = pt41 - (pt45*pt51)/(pt55 + r11);
    ph42 = pt42 - (pt45*pt52)/(pt55 + r11);
    ph43 = pt43 - (pt45*pt53)/(pt55 + r11);
    ph44 = pt44 - (pt45*pt54)/(pt55 + r11);
    ph45 = pt45 - (pt45*pt55)/(pt55 + r11);

    ph51 = -pt51*(pt55/(pt55 + r11) - 1);
    ph52 = -pt52*(pt55/(pt55 + r11) - 1);
    ph53 = -pt53*(pt55/(pt55 + r11) - 1);
    ph54 = -pt54*(pt55/(pt55 + r11) - 1);
    ph55 = -pt55*(pt55/(pt55 + r11) - 1);
    return xh5*9.3 - 180;
}

#endif
