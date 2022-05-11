#include "stm32l476xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <stdbool.h>
//#include "LCD.h"

void Clock_Initialization(void);
void GPIO_Initialization(void);
void SysTick_Initialization(void);
void SysTick_Handler(void);
void LCD_Initialization();
void sortFloors(void);
void delay(int a);
int keypad_scan(void);
void setDirection(void);
void readButton(void);
void gotoPing(void);
void fulfillRequests(void);
void motorOnUp(void);
void motorOnDown(void);
void motorOff(void);
void readRequests(void);
void setDirection(void);
void keypad(void);

int currentFloor = 1;
int direction;
int floorReq[7];
int time = 1000;
int counter;
int desiredFloor;
int floorOrder[7];
int duty_cycle;
int lastFloor;
bool above = false;
bool below = false;


int main(void){

Clock_Initialization();
SysTick_Initialization();
GPIO_Initialization();

  //Testing Functions
  // while(1){
	//   readRequests();
  // }
  // while(1){
	//   readButton();
	//   setDirection();
	//   gotoPing();
  // }



  duty_cycle = 10; //duty cycle to make it easy to get to the top floor
  while(1){ //motor goes up until top button is pressed, then breaks out of loop
	  motorOnUp();
	  if((GPIOB->IDR & 0x8000) != 0x00) {
		  motorOff();
		  break;
	  }
  }
  duty_cycle = 22; //sets the duty cycle to 24, our closest approximation for the distance to go up a floor
  lastFloor = 8;
  currentFloor = 7; //the elevator should be at the top, so the currentFloor is equal to 7

  while(1){
    readButton();
    //keypad();
    setDirection();
    gotoPing();
    // fulfillRequests();
  }
}

void Clock_Initialization(void){
  RCC->CR |= RCC_CR_MSION;//what does this mean? Maybe setting it to lowest part of the RCC register?

  //Select MSI as clock source
  RCC->CFGR &= ~RCC_CFGR_SW;

  // Wait until MSI is ready
  while((RCC->CR & RCC_CR_MSIRDY) == 0);//Makes sure CR is on and ready

  //Clear control register and then set to 8Mhz
  RCC->CR &= ~RCC_CR_MSIRANGE;
  RCC->CR |= RCC_CR_MSIRANGE_7;

  //Set the CR to Final value
  RCC ->CR |= RCC_CR_MSIRGSEL;

  while((RCC->CR & RCC_CR_MSIRDY) == 0);
}

void GPIO_Initialization(void){
  //Enable GPIO ports A, B, C, D, E
  RCC->AHB2ENR |= 0x1F;

  //ALL INPUTS
  //Set inputs according to google doc (Shared)
  GPIOA->MODER &= ~(0xCFF);
  GPIOB->MODER &= ~(0xFF00FFFF); //edited to allow for extra sensors if need be
  GPIOC->MODER &= ~(0x33000);
  GPIOD->MODER &= ~(0x33330000);
  GPIOE->MODER &= ~(0x3000);

  //MOTOR
  //Motor outputs for PC4, PC5
  GPIOC->MODER &= ~(0xA00);
  GPIOC->MODER |= 0x500;
  GPIOA->MODER &= ~(0x2000);
  GPIOA->MODER |= 0x1000;

  //Set motor outputs to push pull
  GPIOC->OTYPER &= ~(0xC000);

  //Set motor outputs to no pull up no pull fulfillDown
  GPIOC->PUPDR &= ~(0xF0000000);
  //KEYPAD
  //Keypad outputs
  GPIOE->MODER &= ~(0x0FF00000);
  GPIOE->MODER |= 0x05500000;

  //Set keypad outputs to open drain
  GPIOE->OTYPER |= 0x3C00;

}

void SysTick_Initialization(void){
  //Systick Timer Setups
  //Disable counter, request, and select external Clock
  SysTick->CTRL &= ~(0x07);

  //Set load to be 10ms
  SysTick->LOAD = 999;

  //Clear current value of VAL register
  SysTick->VAL = 0;

  //Set interrupt priority
  NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);

  //Enable interrupt setting and timer
  SysTick->CTRL |= 0x03;
}

void SysTick_Handler(void){
  counter = counter -1;
  time = time - 1;
}

void delay(int a){
  counter = a;
  while(counter > 0) {
    //do nothing
  }
}

void readButton(void){
  if((GPIOB->IDR & 0x8000) != 0x00) floorReq[6] = -1; //7D
  if((GPIOB->IDR & 0x2000) != 0x00) floorReq[5] = 1;       //6U
  if((GPIOB->IDR & 0x2) != 0x00) floorReq[5] = -1;     //6D
  if((GPIOC->IDR & 0x100) != 0x00) floorReq[4] = 1;      //5U
  if((GPIOC->IDR & 0x40) != 0x00) floorReq[4] = -1;     //5D
  if((GPIOD->IDR & 0x4000) != 0x00) floorReq[3] = 1;      //4U
  if((GPIOD->IDR & 0x1000) != 0x00) floorReq[3] = -1;     //4D
  if((GPIOD->IDR & 0x400) != 0x00) floorReq[2] = 1;      //3U
  if((GPIOD->IDR & 0x100) != 0x00) floorReq[2] = -1;     //3D
  if((GPIOB->IDR & 0x4000) != 0x00) floorReq[1] = 1;      //2U
  if((GPIOB->IDR & 0x1000) != 0x00) floorReq[1] = -1;     //2D
  if((GPIOB->IDR & 0x1) != 0x0000) floorReq[0] = -1;
}

void gotoPing(void){
  below = false;
  above = false;
  for(int i = 0; i <= 6; i++){
    if(floorReq[i] != 0){
      if(currentFloor < i+1){
        above = true;
      } else
      if(currentFloor > i+1){
        below = true;
      }
    }
  }
  if ((direction == 0) && above && below){
	  int distance = 6;
	  for(int i = 0; i <= 6; i++){
	     if(floorReq[i] != 0){
	    	if (abs(currentFloor-(i+1))<distance){
	    		distance =(i+1)-currentFloor;
	    	}
	     }
	  }
	  if (distance>0) {
		motorOnUp();
	  } else {
		  motorOnDown();
	  }
  } else if ((direction == 0) && above){
	  motorOnUp();
  } else if ((direction == 0) && below){
	  motorOnDown();
  } else if((direction == 1) && above){
	  motorOnUp();
  } else if((direction == -1) && below){
	  motorOnDown();
  } else {
      if(direction == 1){
        direction = -1;
      } else if(direction == -1){
        direction = 1;
      }
  }
  if(floorReq[currentFloor -1] != 0){
    if((floorReq[currentFloor -1] == direction)||(currentFloor==7)||(currentFloor==1)){
        floorReq[currentFloor -1] = 0;
        motorOff();
        delay(3000);
    }
  }
  motorOff();
}

void keypad() {
  counter = 3000 ;
  while(counter > 0){
    int desiredFloor = keypad_scan() - '0' ;
    if (desiredFloor != currentFloor) {
      if ((currentFloor > desiredFloor) && (direction == 1)) {
        floorReq[desiredFloor-1] = -1 ;
      } else if ((currentFloor < desiredFloor) && (direction == -1)) {
        floorReq[desiredFloor-1] = 1 ;
      } else {
        floorReq[desiredFloor-1] = direction ;
      }
    }
  }
}

int keypad_scan(void) {
	int row, col;
		int key;

		int keymap[4][4] = {
			{1,2,3,0},
			{4,5,6,0},
			{7,8,9,0},
			{0,0,0,0}
		};

		uint32_t inputMask  = GPIO_IDR_ID1 | GPIO_IDR_ID2 | GPIO_IDR_ID3 | GPIO_IDR_ID5;
		uint32_t outputs[4] = {GPIO_ODR_OD10, GPIO_ODR_OD11, GPIO_ODR_OD12, GPIO_ODR_OD13};
		uint32_t inputs[4]  = {GPIO_IDR_ID1, GPIO_IDR_ID2, GPIO_IDR_ID3, GPIO_IDR_ID5};

		GPIOE->ODR = 0;

		delay(3); // Delay is needed due to the capacitors connected to pin PA 1, 2, 3, 5

		//while( (GPIOA->IDR & inputMask) == inputMask){;}  // Wait until key pressed

		for(row = 0; row < 4; row++){ // Row scan
			GPIOE->ODR |= (GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
			GPIOE->ODR &= ~outputs[row];

			delay(3); // Delay is needed due to the capacitors connected to pin PA 1, 2, 3, 5

			for(col = 0; col < 4; col++){// Column scan
				if((GPIOA->IDR & inputs[col]) == 0 ){
					key = keymap[row][col];
					while( (GPIOA->IDR & inputMask) != inputMask){;} // Wait until key released
					return key;
				}
			}
		}

		return 0;
}

void motorOnUp(void){
  //motor CLEAR
  GPIOC->ODR &= ~(0x30);
  //motor ON and UP
  GPIOC->ODR |= (0x10);

  counter = 1000;

  while(counter > 0){
    if(counter >= duty_cycle*10){
      //enable line to the motor OFF
      GPIOA->ODR &= ~(0x40);
    }
    else{
      //enable line to the motor ON
      if(counter > 0){
        GPIOA->ODR |= (0x40);
      }
    }
  }
  lastFloor = currentFloor;
  currentFloor++;
}

void motorOnDown(void){
  //motor CLEAR
  GPIOC->ODR &= ~(0x30);
  //motor ON and DOWN
  GPIOC->ODR |= (0x20);

  counter = 1000;

  while(counter > 0){
    if(counter >= duty_cycle*10){
      //enable line to the motor OFF
      GPIOA->ODR &= ~(0x40);
    }
    else{
      //enable line to the motor ON
      if(counter > 0){
        GPIOA->ODR |= (0x40);
      }
    }
  }
  lastFloor = currentFloor;
  currentFloor--;
}

void motorOff(void){
  //motor CLEAR
  GPIOC->ODR &= ~(0x30);

  //enable line to motor OFF
  GPIOA->ODR &= ~(0x40);

  //delay(2000);
}

void setDirection(void){
  //if(currentFloor > lastFloor) direction = 1;
  //else if(currentFloor < lastFloor) direction = -1;
  if(currentFloor == 7) direction = -1;
  if(currentFloor == 1) direction = 1;

  int moving = 0;
  for(int i = 0; i <= 6; i++){
	  if(floorReq[i] == 0) moving = 1;
  }
  if(moving == 0) {
	  direction = 0;
    //for(int i = 0; i <= 6; i++){
      //if(floorReq[i] == 0) continue;
      //if(currentFloor - i + 1 > 0) direction = -1;
      //else if(currentFloor - i + 1 < 0) direction = 1;
  }
}
