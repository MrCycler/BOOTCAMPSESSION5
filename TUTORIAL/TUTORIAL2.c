//Registers manage
// set
register |= 0x02;
// reset
register &= ~ (0x02);
//manage a bit
register | bitname;

//date type for registers
uint8_t n;

//Registers for tiva stellaris (PORTS)
//clock in X
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_RX;
while( (SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_RX)==0) { }
// DIR (Input or Output?)
GPIO_PORTF_DIR_R |= 0x02;
// Digital funct Enable
GPIO_PORTF_DEN_R |=0x02; 
// Data register
GPIO_PORTF_DATA_R |=0x02; 
//Eliminate Alternative Funct
GPIO_PORTF_AFSEL_R &= ~(0x10);

// Registers for Outputs
// Current Limit (in mA)
GPIO_PORTF_DR8R_R |=0x02; 

// Registers for Inputs
//Pull up registers
GPIO_PORTF_PUR_R |= 0x10;
//Pull down registers
GPIO_PORTF_PDR_R |= 0x10;

