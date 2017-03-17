#define DP_5G_MASK          0x7000
#define PA5G_BS_MASK        0x0E00
#define PA5G_PW_MASK        0x0180
#define PD_Q5G_MASK         0x0040
#define QI_5G_MASK          0x0038
#define PA_BS_MASK          0x0007

#define PA_CONTROL_DEFAULT  0x4FBD
// PA_CONTROL_MIN, originated from SIRINFPV code.
#define PA_CONTROL_MIN      ((PA_CONTROL_DEFAULT | PD_Q5G_MASK) & (~(PA5G_PW_MASK | PA5G_BS_MASK)))
