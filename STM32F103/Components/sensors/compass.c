
void CompassLED(float HeadingValue);
        if (((HeadingValue < 25.0f)&&(HeadingValue >= 0.0f))||((HeadingValue >=340.0f)&&(HeadingValue <= 360.0f)))
        {
          STM_EVAL_LEDOn(LED10);
          STM_EVAL_LEDOff(LED3);
          STM_EVAL_LEDOff(LED6);
          STM_EVAL_LEDOff(LED7);
          STM_EVAL_LEDOff(LED4);
          STM_EVAL_LEDOff(LED8);
          STM_EVAL_LEDOff(LED9);
          STM_EVAL_LEDOff(LED5);
        }
        else  if ((HeadingValue <70.0f)&&(HeadingValue >= 25.0f))
        {
          STM_EVAL_LEDOn(LED9);
          STM_EVAL_LEDOff(LED6);
          STM_EVAL_LEDOff(LED10);
          STM_EVAL_LEDOff(LED3);
          STM_EVAL_LEDOff(LED8);
          STM_EVAL_LEDOff(LED5);
          STM_EVAL_LEDOff(LED4);
          STM_EVAL_LEDOff(LED7);
        } 
        else  if ((HeadingValue < 115.0f)&&(HeadingValue >= 70.0f))
        {
          STM_EVAL_LEDOn(LED7);
          STM_EVAL_LEDOff(LED3);
          STM_EVAL_LEDOff(LED4);
          STM_EVAL_LEDOff(LED9);
          STM_EVAL_LEDOff(LED10);
          STM_EVAL_LEDOff(LED8);
          STM_EVAL_LEDOff(LED6);
          STM_EVAL_LEDOff(LED5);
        }
        else  if ((HeadingValue <160.0f)&&(HeadingValue >= 115.0f))
        {
          STM_EVAL_LEDOn(LED5);
          STM_EVAL_LEDOff(LED6);
          STM_EVAL_LEDOff(LED10);
          STM_EVAL_LEDOff(LED8);
          STM_EVAL_LEDOff(LED9);
          STM_EVAL_LEDOff(LED7);
          STM_EVAL_LEDOff(LED4);
          STM_EVAL_LEDOff(LED3);
        } 
        else  if ((HeadingValue <205.0f)&&(HeadingValue >= 160.0f))
        {
          STM_EVAL_LEDOn(LED3);
          STM_EVAL_LEDOff(LED6);
          STM_EVAL_LEDOff(LED4);
          STM_EVAL_LEDOff(LED8);
          STM_EVAL_LEDOff(LED9);
          STM_EVAL_LEDOff(LED5);
          STM_EVAL_LEDOff(LED10);
          STM_EVAL_LEDOff(LED7);
        } 
        else  if ((HeadingValue <250.0f)&&(HeadingValue >= 205.0f))
        {
          STM_EVAL_LEDOn(LED4);
          STM_EVAL_LEDOff(LED6);
          STM_EVAL_LEDOff(LED10);
          STM_EVAL_LEDOff(LED8);
          STM_EVAL_LEDOff(LED9);
          STM_EVAL_LEDOff(LED5);
          STM_EVAL_LEDOff(LED3);
          STM_EVAL_LEDOff(LED7);
        } 
        else  if ((HeadingValue < 295.0f)&&(HeadingValue >= 250.0f))
        {
          STM_EVAL_LEDOn(LED6);
          STM_EVAL_LEDOff(LED9);
          STM_EVAL_LEDOff(LED10);
          STM_EVAL_LEDOff(LED8);
          STM_EVAL_LEDOff(LED3);
          STM_EVAL_LEDOff(LED5);
          STM_EVAL_LEDOff(LED4);
          STM_EVAL_LEDOff(LED7);
        }        
        else  if ((HeadingValue < 340.0f)&&(HeadingValue >= 295.0f))
        {
          STM_EVAL_LEDOn(LED8);
          STM_EVAL_LEDOff(LED6);
          STM_EVAL_LEDOff(LED10);
          STM_EVAL_LEDOff(LED7);
          STM_EVAL_LEDOff(LED9);
          STM_EVAL_LEDOff(LED3);
          STM_EVAL_LEDOff(LED4);
          STM_EVAL_LEDOff(LED5);
        }
      }
      else
      {
        /* Toggle All LEDs */
        STM_EVAL_LEDToggle(LED7);
        STM_EVAL_LEDToggle(LED6);
        STM_EVAL_LEDToggle(LED10);
        STM_EVAL_LEDToggle(LED8);
        STM_EVAL_LEDToggle(LED9);
        STM_EVAL_LEDToggle(LED3);
        STM_EVAL_LEDToggle(LED4);
        STM_EVAL_LEDToggle(LED5);
      }
    }




    /*
    if (compassX > compassX_Max) compassX_Max = compassX;    // determine max of x
    if (compassX < compassX_Min) compassX_Min = compassX;    // determine min of x
    if (compassY > compassY_Max) compassY_Max = compassY;    // determine max of y
    if (compassY < compassY_Min) compassY_Min = compassY;    // determine min of y
    if (compassZ > compassZ_Max) compassZ_Max = compassZ;    // determine max of z
    if (compassZ < compassZ_Min) compassZ_Min = compassZ;    // determine min of z
  
    compassX_Range = compassX_Max - compassX_Min;                   // determine range of x
    compassY_Range = compassY_Max - compassY_Min;                   // determine range of y
    compassZ_Range = compassZ_Max - compassZ_Min;                   // determine range of z
    
    compassX_Norm = (compassX - ((compassX_Max + compassX_Min) / 2.0));                     // normalized x vector
    compassY_Norm = (compassY - ((compassY_Max + compassY_Min) / 2.0)) * compassX_Range / compassY_Range; // normalized y vector
    compassZ_Norm = (compassZ - ((compassZ_Max + compassZ_Min) / 2.0)) * compassX_Range / compassZ_Range; // normalized z vector
    
    heading = atan2f(compassX_Norm, compassY_Norm);          // atan2 to calculate heading vector in x-y plane
    if(heading < 0) heading += _2PI;           // heading should be between 0 and 2PI
    else if(heading > _2PI) heading -= _2PI;
    heading *= _180_PI;                        // convert to degrees
    */
    /*
    int i = 0;
    for(i=0;i<3;i++){
        accelerometerBufferFloat[i] = accelerometerBuffer[i]/100.0f;
    }
      fNormAcc = sqrt((accelerometerBufferFloat[0]*accelerometerBufferFloat[0])+(accelerometerBufferFloat[1]*accelerometerBufferFloat[1])+(accelerometerBufferFloat[2]*accelerometerBufferFloat[2]));
      
      fSinRoll = -accelerometerBufferFloat[1]/fNormAcc;
      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
      fSinPitch = accelerometerBufferFloat[0]/fNormAcc;
      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));
      
     if ( fSinRoll >0)
     {
       if (fCosRoll>0)
       {
         RollAng = acos(fCosRoll)*180/PI;
       }
       else
       {
         RollAng = acos(fCosRoll)*180/PI + 180;
       }
     }
     else
     {
       if (fCosRoll>0)
       {
         RollAng = acos(fCosRoll)*180/PI + 360;
       }
       else
       {
         RollAng = acos(fCosRoll)*180/PI + 180;
       }
     }
     
      if ( fSinPitch >0)
     {
       if (fCosPitch>0)
       {
            PitchAng = acos(fCosPitch)*180/PI;
       }
       else
       {
          PitchAng = acos(fCosPitch)*180/PI + 180;
       }
     }
     else
     {
       if (fCosPitch>0)
       {
            PitchAng = acos(fCosPitch)*180/PI + 360;
       }
       else
       {
          PitchAng = acos(fCosPitch)*180/PI + 180;
       }
     }

      if (RollAng >=360)
      {
        RollAng = RollAng - 360;
      }
      
      if (PitchAng >=360)
      {
        PitchAng = PitchAng - 360;
      }
      
      fTiltedX = compassBuffer[0]*fCosPitch+compassBuffer[2]*fSinPitch;
      fTiltedY = compassBuffer[0]*fSinRoll*fSinPitch+compassBuffer[1]*fCosRoll-compassBuffer[1]*fSinRoll*fCosPitch;
      
      heading = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;
      
      if (heading < 0)
      {
        heading = heading + 360;    
      }
    */