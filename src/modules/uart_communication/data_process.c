/*
 * data_process.c
 *
 *  Created on: 2016-1-21
 *      Author: gyzhang
 */
#include <stdio.h>
#include<modules/uart_communication/data_process.h>
void force_sensor_data_decode(float* resultData, char* buff)
{
	uint8_t i;
	uint8_t Index;
	uint8_t k ;
	uint16_t ADcounts[6];
	float m_dResultChValue[6];
	float m_dGain[6];
	float m_dAmpZero[6];
	float decoupled[6][6]=
		{
      {0.22416,-0.12460,0.50932,15.04759,0.02498,14.42661},
			{ -0.16640,-17.15939,0.00455,8.64643,0.24355,-8.47881},
			{45.52190,-0.11947,45.53771,-0.15801,45.71040,0.98397},
			{0.00434,-0.00848,0.24087,0.00237,-0.20537,-0.00358},
			{-0.26570,-0.00259,0.12538,-0.00135,0.12824,-0.00278},
			{0.00108,0.10295,0.00315,0.08529,0.00047,-0.09361}
	    };

		m_dGain[0] = 123.519878;
		m_dGain[1] = 123.453219;
		m_dGain[2] = 123.580741;
		m_dGain[3] = 123.490896;
		m_dGain[4] = 123.395254;
		m_dGain[5] = 123.554657;

	  m_dAmpZero[0] = 32715.000000;
      m_dAmpZero[1] = 32693.000000;
		m_dAmpZero[2] = 32689.000000;
		m_dAmpZero[3] = 32681.000000;
		m_dAmpZero[4] = 32703.000000;
		m_dAmpZero[5] = 32735.000000;

	if((buff[0] == 0xAA) && (buff[1] == 0x55))
		{
			Index = 6;
			char CheckSum = 0x00;
		    for(i = 0x00;i < 0x0c;i++)
			{
				CheckSum += buff[Index];//
				Index++;
			}
			if(CheckSum == buff[Index])//
			{

                Index = 6;

                for(k = 0x00;k < 6; k++)
                {
                    ADcounts[k] = buff[Index]*256 + buff[Index+1]; //AD
                    m_dResultChValue[k] = (float)(1000.0f*(ADcounts[k]-m_dAmpZero[k])/65535.0f*5.0f/m_dGain[k]/2.5f);
                    Index = Index + 2;
                }
                for(i = 0x00;i < 6; i++)
                {
                	resultData[i] = m_dResultChValue[0]*decoupled[i][0]+m_dResultChValue[1]*decoupled[i][1]
			                   +m_dResultChValue[2]*decoupled[i][2]+m_dResultChValue[3]*decoupled[i][3]
			                   +m_dResultChValue[4]*decoupled[i][4]+m_dResultChValue[5]*decoupled[i][5];
                }
				resultData[0]=resultData[0]+2.1482f;
                resultData[1]=resultData[1]+1.1133f;
                resultData[2]=resultData[2]+1.1122f;
                resultData[3]=resultData[3]+0.0261f;
                resultData[4]=resultData[4]+0.022f;
                resultData[5]=resultData[5]+0.0208f;
			}

		}

}


