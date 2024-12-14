#include "STM32G4xx_TIMER.hpp"

PWM::PWM(TIMERBase tim_X, uint32_t frequency, uint8_t GPIO_OC_X, PWMPolarity OC_polarity, 
         uint8_t GPIO_OCN_X, PWMPolarity OCN_polarity, uint8_t dead_time)
{

    state = TIMER_BUSY;

    TIMER_Base = reinterpret_cast<TIMER_OC_TypeDef*>(tim_X);

    GPIO _OC(GPIO_OC_X, GPIO::ALTFN, GPIO::PUSHPULL, GPIO::SPEED_VERYHIGH, GPIO::FLOAT);

    GPIO _OCN;

    if(GPIO_OCN_X != 0xFF)
    {
        _OCN.Init(GPIO_OCN_X, GPIO::ALTFN, GPIO::PUSHPULL, GPIO::SPEED_VERYHIGH, GPIO::FLOAT);
    }

    switch(tim_X)
    {
    case TIM1:

        _OC.SetAltFn(TIM1_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM1_PWM_AltFn);
        }

        RCC->APB2ENR.TIM1EN = 1;

        break;

    case TIM2:

        _OC.SetAltFn(TIM2_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM2_PWM_AltFn);
        }

        RCC->APB1ENR.TIM2EN = 1;

        break;
    
    case TIM3:
    
        _OC.SetAltFn(TIM3_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM3_PWM_AltFn);
        }

        RCC->APB1ENR.TIM3EN = 1;

        break;
        
    case TIM4:

        _OC.SetAltFn(TIM4_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM4_PWM_AltFn);
        }

        RCC->APB1ENR.TIM4EN = 1;

        break;

    case TIM5:

        _OC.SetAltFn(TIM5_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM5_PWM_AltFn);
        }

        RCC->APB1ENR.TIM5EN = 1;

        break;
    
    case TIM6:
        
        state = TIMER_ERROR;

        return;

    case TIM7:

        state = TIMER_ERROR;

        return;

        break;

    case TIM8:

        _OC.SetAltFn(TIM8_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM8_PWM_AltFn);
        }

        RCC->APB2ENR.TIM8EN = 1;

        break;
    
    case TIM9:

        _OC.SetAltFn(TIM9_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM9_PWM_AltFn);
        }

        RCC->APB2ENR.TIM9EN = 1;

        break;

    case TIM10:

        _OC.SetAltFn(TIM10_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM10_PWM_AltFn);
        }

        RCC->APB2ENR.TIM10EN = 1;

        break;

    case TIM11:

        _OC.SetAltFn(TIM11_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM11_PWM_AltFn);
        }

        RCC->APB2ENR.TIM11EN = 1;

        break;
    
    case TIM12:

        _OC.SetAltFn(TIM12_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM12_PWM_AltFn);
        }

        RCC->APB1ENR.TIM12EN = 1;

        break;

    case TIM13:

        _OC.SetAltFn(TIM13_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM13_PWM_AltFn);
        }

        RCC->APB1ENR.TIM13EN = 1;

        break;

    case TIM14:
        
        _OC.SetAltFn(TIM14_PWM_AltFn);

        if(GPIO_OCN_X != 0xFF)
        {
            _OCN.SetAltFn(TIM14_PWM_AltFn);
        }

        RCC->APB1ENR.TIM14EN = 1;

        break;
    }

    TIMER_Base->CR1.CEN = 0;

    TIMER_Base->CR1.CKD = 0;

    TIMER_Base->CR1.ARPE = 1;

    TIMER_Base->CR1.CMS = 0;

    TIMER_Base->CR1.DIR = 0;

    TIMER_Base->CR1.OPM = 0;

    TIMER_Base->PSC.PSC = 0;

    TIMER_Base->ARR.ARR = (System_Clock_Freq / frequency) - 1;

    TIMER_Base->CCMR1_O.OC1M = 6; // Set mod as PWM

    TIMER_Base->CCMR1_O.OC1PE = 1;

    TIMER_Base->CCER.CC1P = OC_polarity;

    if(GPIO_OCN_X != 0xFF)
    {
       TIMER_Base->CCER.CC1NP = OCN_polarity;     
    }

    TIMER_Base->BDTR.BKP = 1;
    
    state = TIMER_READY;
}

void PWM::setDuty(uint8_t duty_cycle)
{
    if(duty_cycle >100 || state != TIMER_READY)
    {
        state = TIMER_ERROR;

        return;
    }

    TIMER_Base->CCR1.CCR1 = (TIMER_Base->ARR.ARR +1) * duty_cycle / 100;
}

void PWM::PWMStart()
{
	TIMER_Base->BDTR.MOE = 1;
	TIMER_Base->CCER.CC1E = 1;
	TIMER_Base->CR1.CEN = 1;

}

void PWM::PWMNStart()
{
	TIMER_Base->BDTR.MOE = 1;
    TIMER_Base->CCER.CC1NE = 1;
    TIMER_Base->CR1.CEN = 1;
}

void PWM::PWMStop()
{
	TIMER_Base->BDTR.MOE = 0;
    TIMER_Base->CCER.CC1E = 0;
    TIMER_Base->CR1.CEN = 0;
}

void PWM::PWMNStop()
{
	TIMER_Base->BDTR.MOE = 0;
    TIMER_Base->CCER.CC1NE = 0;
    TIMER_Base->CR1.CEN = 0;
}
