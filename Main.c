/*
 * Definicoes do compilador
 */
#include <string.h>

/*
 * Definicoes - Biblioteca Comum
 */


/*
 * Rotinas - Biblioteca Comum
 */

/*
 * Definicoes - Biblioteca
 */
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/DefinicoesHALB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/DefinicoesTiposDadosB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/DefinicoesProjetoB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/DefinicoesSerialB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/DefinicoesStringB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/DefinicoesTimersB.h"

/*
 * Rotinas - Biblioteca
 */
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/RotinasInicializacaoB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/RotinasTimersB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/RotinasGerenciamentoFilasB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/RotinasLedsTecladoB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/RotinasSerialB.h"
#include "../../../../BibliotecaWifi/trunk/ArquivosBiblioteca/RotinasConversoesB.h"

/*
 * Definicoes - Projeto
 */
#include "DefinicoesDimerizacao.h"
 
/*
 * Rotinas - Projeto
 */



/**
 * Trava de seguranca para garantir que o projeto correto seja selecionado em DefinicoesHAL.h
 */
#if (!defined(CW6_WIFI))

   #error "Defina o projeto correto"

#endif

/*
 * Trava de seguranca para garantir que foi definido o tipo de compilacao correto para o projeto em DefinicoesProjetosB.h
 */
#if (!defined(COMPILACAO_FIRMWARE))

    #error "Defina o tipo de compilacao correto"

#endif

/*
 * variável usada para fazer um mapeamento do estado do programa, setando bit em determinada parte e se o fluxo estiver correto
 * vai limpando os bits. Portanto vai limpar o Watchdog somente se a variável estiver com valor zero (0), ou seja, tudo estável
 */
BYTE trilhaWDT;

/*
 * Rotina que analisa a interrupcao 
 */
static void interrupt isr (void)
{
    /*
     * Variaveis locais
     */
    BYTE auxISR;
    BYTE auxByteEntradaSerial;
    
    WORD temp;
    WORD resultado;

    /*
     * Variaveis externas
     */
    extern volatile BIT flagTemSync;
    extern volatile BIT flagCalculando;
    
    extern BIT flagFrequencia60Hz;
    extern BIT flagVoltagem220V;
    extern BIT flagEstadoComparador;
    extern BIT flagBordaDescida;
    
    extern volatile BIT flagIntborda;

    extern volatile BYTE timerTemSync;
    extern volatile BYTE limite;    
    extern volatile BYTE limiteAtual;
    extern volatile BYTE limiteFuturo;
    extern volatile BYTE indiceTemposDisparo;
    extern volatile BYTE mascara_disparo[NUMERO_CANAIS_DIMERIZAVEIS + 1];
    extern volatile BYTE mascara_disparo_futuro[NUMERO_CANAIS_DIMERIZAVEIS + 1];
    
    extern BYTE filaSaidaSerial[MAX_FILA_SAIDA_SERIAL];
    extern BYTE iniFilaSaidaSerial;
    extern BYTE fimFilaSaidaSerial;
    extern BYTE filaEntradaSerial[MAX_FILA_ENTRADA_SERIAL];
    extern BYTE fimFilaEntradaSerial;
    extern BYTE auxFimFilaEntradaSerial;
    extern BYTE timerInterpretaComandoEntradaSerial;
    extern BYTE debounceRedeEletrica;
    extern BYTE flagsCanaisDimerizando;
    
    extern volatile WORD tempoSemiCiclo;
    extern volatile WORD temposDisparoAtual[NUMERO_CANAIS_DIMERIZAVEIS + 1];
    extern volatile WORD temposDisparoFuturo[NUMERO_CANAIS_DIMERIZAVEIS + 1];
    
    extern WORD ultimoTempo;
    extern WORD tempoMeioCiclo;
    extern WORD tempoReferencia;
    
    extern ESTADO_CALIBRACAO estadoCalibracao;
    
    extern TIMER1 timer1;
    
    extern COMPARADOR comparador;
    
    extern ENUM_COMANDO_AT_ENVIADO estadoEnviaComandoAT;

    
    /*
     * Interrupcao da dimerizacao dos leds, leitura das teclas e tick do sistema
     */
    if (TMR4IF)
    {
        AnalisaInterrupcaoLeds_B();
    }
    

    /**
     * Interrupcao da borda de descida da rede eletrica
     * (Devido ao hardware existe uma pequena diferenca de fase entre o sinal da rede eletrica e o sinal da placa)
     *
     * Esta interrupção ocorre a cada ciclo completo da rede eletrica
     * Para 60hz o tempo é de 16,666 milisegundos
     */
    if ((INTF) && (INTE))
    {
        INTF = FALSE;
        TMR1ON = FALSE;
        temp = timer1.valor;
        TMR1ON = TRUE;
        
        
        switch (estadoCalibracao)
        {
            case DETECTANDO_FREQUENCIA:
                
                timer1.valor = 0;
                
                /*
                 * Observacao 1: Quanto maior a frequencia, menor o periodo
                 */                
                if ((temp > PERIODO_FREQUENCIA_MAXIMA_HZ) && (temp < PERIODO_FREQUENCIA_MINIMA_HZ))
                {
                    if (debounceRedeEletrica)
                    {
                        debounceRedeEletrica--;
                    }
                    
                    if (debounceRedeEletrica == 0)
                    {
                        
                        estadoCalibracao = DETECTANDO_VOLTAGEM;
                        
                        debounceRedeEletrica = CICLOS_MEIO_SEGUNDO_60HZ;
                        
                        if (flagFrequencia60Hz)
                        {
                            tempoSemiCiclo = TEMPO_SEMICICLO_60_HZ;
                        }
                        else
                        {
                            tempoSemiCiclo = TEMPO_SEMICICLO_50_HZ;
                        }
                        
                    }
                    else
                    {
                        if (flagFrequencia60Hz)
                        {
                            if (temp > PERIODO_FREQUENCIA_MEDIA_HZ)//se tempo maior do que tempo para 55hz então muda detecção para 50hz
                            {
                                flagFrequencia60Hz = FALSE;
                                debounceRedeEletrica = CICLOS_1_SEGUNDO_60HZ;
                            }
                        }
                        else
                        {
                            if (temp <= PERIODO_FREQUENCIA_MEDIA_HZ)//se está detectando para 50hz e encontra tempo maior que 55hz então reseta debounce e altera detecção para 60 Hz
                            {
                                flagFrequencia60Hz = TRUE;
                                debounceRedeEletrica = CICLOS_1_SEGUNDO_60HZ;
                            }
                        }
                    }
                }
                else
                {
                    debounceRedeEletrica = CICLOS_1_SEGUNDO_60HZ;
                }
                
            break;
                
            case DETECTANDO_VOLTAGEM:
                //para 110V - valor entre descida e subida é de 7420
                //para 220V - valor entre descida e subida é de 7760
                if (debounceRedeEletrica)
                {
                    debounceRedeEletrica--;
                    
                    if (INTEDG == 1u)//se pegou borda de subida então verifica tempo
                    {
                        if (flagVoltagem220V)
                        {
                            if (temp < CONSTANTE_220_V)
                            {
                                flagVoltagem220V = FALSE;
                                debounceRedeEletrica = CICLOS_MEIO_SEGUNDO_60HZ;
                            }
                        }
                        else
                        {
                            if (temp >= CONSTANTE_220_V)
                            {
                                flagVoltagem220V = TRUE;
                                debounceRedeEletrica = CICLOS_MEIO_SEGUNDO_60HZ;
                            }
                        }
                    }
                    
                    INTEDG ^= 1;
                    
                    if (debounceRedeEletrica == 0u)//se ainda não zerou contador deixa reiniciar o timer para a próxima referencia
                    {
                        if (INTEDG == 1u)
                        {
                            INTEDG = 0;
                            timer1.valor = 0;
                        }
                        estadoCalibracao = MOSTRA_LEDS_VOLTAGEM;
                        debounceRedeEletrica = CICLOS_1_SEGUNDO_60HZ;
                        flagTemSync = TRUE;
                    }
                    else
                    {
                        timer1.valor = 0;
                    }
                }
                
            break;
                
            case MOSTRA_LEDS_VOLTAGEM:
                
                if (debounceRedeEletrica)
                {
                    debounceRedeEletrica--;
                    
                    if (debounceRedeEletrica == 0u)
                    {
                        estadoCalibracao = ESTABILIZADO;
                        flagEstadoComparador = TRUE;
                        TMR1ON = FALSE;
                        timer1.valor = 0;
                        ultimoTempo = 0;
                        TMR1ON = TRUE;
                        CCP1IE = FALSE;
                        CCP1IF = FALSE;
                    }
                }
                    
            break;
                
            case ESTABILIZADO:

                if (temp > ultimoTempo)
                {
                    resultado = temp - ultimoTempo;
                }
                else
                {
                    resultado = (OVERFLOW_TIMER1 - ultimoTempo) + temp;
                }

                /*
                 * Observacao 1: Quanto maior a frequencia, menor o periodo
                 */                
                if ((resultado > PERIODO_FREQUENCIA_MAXIMA_HZ) && (resultado < PERIODO_FREQUENCIA_MINIMA_HZ)) //testa se está na faixa de 48hz à 62hz
                {                    
                    timerTemSync = TEMPO_2S_TICK;
                    
                    //testa se está no estadoComparador == 1. Se estiver no estado zero pode ser que seja apenas ruído, então ignora
                    if (flagEstadoComparador)
                    {
                        tempoMeioCiclo = resultado;
                        ultimoTempo = temp;
                        flagEstadoComparador = FALSE;
                        INTEDG = 1;
                        comparador.valor_disparo = (ultimoTempo + 100); // Programando disparo do comparador para 100uS
                        CCP1IF = FALSE;
                        CCP1IE = TRUE;
                        
                    }
                    
                }
                
            break;
        }
    }
    
    
    /*
     * Interrupcao por comparador
     * Toda vez que o comparador coincide, chama esta parte
     * Comparador é responsável por fazer a dimerização das cargas no programa
     * Ligando e desligando dentro de um semiciclo a saida do triac
     */
    if ( (CCP1IF) && (CCP1IE) )
    {
        CCP1IF = FALSE;
        
        if (estadoCalibracao == ESTABILIZADO)
        {
            if (flagEstadoComparador == FALSE)//se chegou aqui validou borda
            {                
                tempoReferencia = ultimoTempo;              
                flagBordaDescida = TRUE;
                flagEstadoComparador = TRUE;
                INTEDG = 0;
                
                /*
                 * Se existirem canais para dimerizar 
                 */                
                if (flagsCanaisDimerizando)
                {  
                    flagIntborda = TRUE;
                }
                
                if (flagCalculando == FALSE)
                {
                    
                    limiteAtual = limiteFuturo;
                    
                    for (auxISR = 0;auxISR <= NUMERO_CANAIS_DIMERIZAVEIS; auxISR++)
                    {
                        temposDisparoAtual[auxISR] = temposDisparoFuturo[auxISR];
                        mascara_disparo[auxISR] = mascara_disparo_futuro[auxISR];                        
                    }
                }
                
                limite = limiteAtual;
                indiceTemposDisparo = 0;
                CCP1IE = FALSE;
                comparador.valor_disparo = tempoReferencia + temposDisparoAtual[0]; 
                CCP1IE = TRUE;
                CCP1IF = FALSE;
            }
            else
            {
                if (limite)
                {
                    limite--;                          
                }

                if (limite == 0u)
                {
                    CCP1IE = 0;
                    //Feito desta forma para nao afetar SENTIDO_SERIAL
                    LATA = LATA & 0xF0;
                    
                    if (flagBordaDescida)
                    {
                        flagBordaDescida = FALSE;
                        limite = limiteAtual;
                        tempoReferencia = tempoReferencia + (tempoMeioCiclo / 2);
                        indiceTemposDisparo = 0;
                        comparador.valor_disparo = tempoReferencia + temposDisparoAtual[0];
                        CCP1IE = TRUE;
                        CCP1IF = FALSE;
                    }
                }
                else
                {
                    auxISR = mascara_disparo[indiceTemposDisparo];

                    LATA = (LATA & 0xF0) | (auxISR & 0x0F); // Liga canais

                    indiceTemposDisparo++;

                    CCP1IE = FALSE;
                    comparador.valor_disparo = tempoReferencia + temposDisparoAtual[indiceTemposDisparo];
                    CCP1IF = FALSE;
                    CCP1IE = TRUE;                    
                }
            }            
        }            
    }
    
    
    /*
     * Interrupcao para interpretar comandos da entrada serial RS232
     */
    if ( (TMR6IF) && (TMR6IE) )
    {
        TMR6IF = FALSE;
        
        if (timerInterpretaComandoEntradaSerial)
        {
            timerInterpretaComandoEntradaSerial--;
            if (timerInterpretaComandoEntradaSerial == 0u)
            {
                fimFilaEntradaSerial = auxFimFilaEntradaSerial;
                TMR6IE = FALSE;
            }
        }
    }
    
    
    /*
     * Entrada da Serial RS232
     */
    if ((SERIAL_RCIF) && (SERIAL_RCIE))
    {
        if ( (SERIAL_FERR) || (SERIAL_OERR) )
        {
            if (SERIAL_FERR)
            {
                auxByteEntradaSerial = SERIAL_RX_REG;
            }

            if (SERIAL_OERR)
            {
                SERIAL_CREN = FALSE;

                auxByteEntradaSerial = SERIAL_RX_REG;
                auxByteEntradaSerial = SERIAL_RX_REG;

                SERIAL_CREN = TRUE;
            }
            
            auxFimFilaEntradaSerial = fimFilaEntradaSerial;
            timerInterpretaComandoEntradaSerial = 0;
        }
        else
        {
            auxByteEntradaSerial = SERIAL_RX_REG;

            if ( (estadoEnviaComandoAT != COMANDO_AT_ENVIADO_CONFIGURA_BAUD_RATE) && (RESET_ESP) )
            {
                filaEntradaSerial[auxFimFilaEntradaSerial] = auxByteEntradaSerial;
                IncrementaFilaCircular_B(auxFimFilaEntradaSerial, MAX_FILA_ENTRADA_SERIAL);

                TMR6 = 0;
                TMR6IF = FALSE;
                TMR6IE = TRUE;
                timerInterpretaComandoEntradaSerial = TEMPO_1mS_ENTRADA_SERIAL;
            }
        }
    }
    
    
    /*
     * Saida da Serial
     */
    if (SERIAL_TXIF)
    {
        SERIAL_TXIF = FALSE;

        if (SERIAL_TXIE)
        {            
            SERIAL_TX_REG = filaSaidaSerial[iniFilaSaidaSerial];
            iniFilaSaidaSerial++;
            
            if (iniFilaSaidaSerial == fimFilaSaidaSerial)
            {
                SERIAL_TXIE = FALSE;
                
                iniFilaSaidaSerial = 0;
                fimFilaSaidaSerial = 0;
            }
        }
    }
}

void main(void) 
{
    /*
     * Variaveis externas
     */
    extern BIT flagModoCentral;
    
    
    Inicializacao_B(TRUE);

    /*
     * Loop Principal
     */
    trilhaWDT = 0;
    for(;;)
    {
        if (flagModoCentral)
        {
            if (trilhaWDT == 0u)
            {
                CLRWDT();
            }
        }
        else
        {
            if (trilhaWDT == 0x0F)
            {
                trilhaWDT = 0x00;
                CLRWDT();
            }
        }
            
        
        TarefasNaoDependentesTick_B();
    }
}

