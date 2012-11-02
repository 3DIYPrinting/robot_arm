#include "stag.h"

using namespace std;


// Global configuration

int force_sixaxis = 1;
int force_ds3 = 0;
int timestamp = 0;
int nostdin = 0;

void fatal(char *msg) {
    if (errno) perror(msg);
    else fprintf(stderr, "%s\n", msg);
    exit(1);
}


void be_memcpy(const void *destination, const void *source, uint8_t size)
{
    uint8_t copy_count = 0;
    while (size) ((uint8_t *) destination)[copy_count++] = ((uint8_t *) source)[--size];
}


const char *myba2str(const bdaddr_t *ba) {
    static char buf[2][18]; // Static buffer valid for two invocations.
    static int index = 0;
    index = (index + 1) % 2;
    sprintf(buf[index], "%02x:%02x:%02x:%02x:%02x:%02x",
            ba->b[5], ba->b[4], ba->b[3], ba->b[2], ba->b[1], ba->b[0]);
    return buf[index];
}


int l2cap_listen(unsigned short psm) {
    int sk = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sk < 0) fatal("socket");

    struct sockaddr_l2 addr;
    memset(&addr, 0, sizeof (addr));
    addr.l2_family = AF_BLUETOOTH;
    addr.l2_bdaddr = {
        {0, 0, 0, 0, 0, 0}};
    addr.l2_psm = htobs(psm);
    if (bind(sk, (struct sockaddr *) &addr, sizeof (addr)) < 0) {
        perror("bind");
        close(sk);
        return -1;
    }

    if (listen(sk, 5) < 0) fatal("listen");
    return sk;
}

struct motion_dev *accept_device(int csk, int isk) {
    fprintf(stderr, "Incoming connection...\n");
    struct motion_dev *dev = (motion_dev*) malloc(sizeof (struct motion_dev));
    if (!dev) fatal("malloc");

    dev->csk = accept(csk, NULL, NULL);
    if (dev->csk < 0) fatal("accept(CTRL)");
    dev->isk = accept(isk, NULL, NULL);
    if (dev->isk < 0) fatal("accept(INTR)");

    struct sockaddr_l2 addr;
    socklen_t addrlen = sizeof (addr);
    if (getpeername(dev->isk, (struct sockaddr *) &addr, &addrlen) < 0)
        fatal("getpeername");
    dev->addr = addr.l2_bdaddr;

    {
        // Distinguish SIXAXIS / DS3 / PSMOVE.
        unsigned char resp[64];
        char get03f2[] = {HIDP_TRANS_GET_REPORT | HIDP_DATA_RTYPE_FEATURE | 8,
            0xf2, sizeof (resp), sizeof (resp) >> 8};
        send(dev->csk, get03f2, sizeof (get03f2), 0); // 0301 is interesting too.
        int nr = recv(dev->csk, resp, sizeof (resp), 0);
        if (nr < 19) dev->type = PSMOVE;
        else if (force_sixaxis) dev->type = SIXAXIS;
        else if (force_ds3) dev->type = DS3;
        else dev->type = (resp[13] == 0x40) ? SIXAXIS : DS3; // My guess.
    }

    return dev;
}


int l2cap_connect(bdaddr_t *ba, unsigned short psm) {
    int sk = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sk < 0) fatal("socket");

    struct sockaddr_l2 daddr;
    memset(&daddr, 0, sizeof (daddr));
    daddr.l2_family = AF_BLUETOOTH;
    daddr.l2_bdaddr = *ba;
    daddr.l2_psm = htobs(psm);
    if (connect(sk, (struct sockaddr *) &daddr, sizeof (daddr)) < 0)
        fatal("connect");

    return sk;
}


#define IR0 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00, 0x41
#define IR1 0x40, 0x00

#define BIT1 2

void dump(const char *tag, const unsigned char *buf, int len) {
    fprintf(stderr, "%s[%d]", tag, len);
    for (int i = 0; i < len; ++i) fprintf(stderr, " %02x", buf[i]);
    fprintf(stderr, "\n");
}

void hidp_trans(int csk,int isk, const char *buf, int len) {
    
    if (send(csk, buf, len, 0) != len) fatal("send(CTRL)");
    unsigned char ack;
    //int nr = recv(csk, &ack, sizeof (ack), 0);
    int nr = recv(csk, &ack, sizeof (ack), MSG_DONTWAIT);
    //if (nr != 1 || ack != 0) fatal("ack");
}
//neeeds work
void setup_device(struct motion_dev *dev) {
   
    switch (dev->type) {
        case SIXAXIS:
        case DS3:
        {
            //printf("setup ds3\n");
            // Enable reporting
            char set03f4[] = {HIDP_TRANS_SET_REPORT | HIDP_DATA_RTYPE_FEATURE, 0xf4,
                0x42, 0x03, 0x00, 0x00};

            //char setup[] = {0x21, 0x09, 0x03,0xF4, 0x00,0x00,
            //    0x42, 0x03, 0x00, 0x00, 0x00, 0x04};
            
            
            hidp_trans(dev->csk,dev->isk, set03f4, sizeof (set03f4));
            
            //hidp_trans(dev->csk,dev->isk, setup, sizeof (setup));
            
            //printf("finish trans\n");
            // Leds: Display 1+index in additive format.
            static const char ledmask[10] = {1, 2, 4, 8, 6, 7, 11, 13, 14, 15};
//#define LED_PERMANENT 0xff, 0x27, 0x00, 0x00, 0x32
            #define LED_PERMANENT 0xff, 0x27, 0x10, 0x00, 0x32
            char set0201[] = {
                HIDP_TRANS_SET_REPORT | HIDP_DATA_RTYPE_OUTPUT, 0x01,
                0x00, 0x00, 0x00, 0, 0, 0x00, 0x00, 0x00,
                0x00, ledmask[dev->index % 10] << 1,
                LED_PERMANENT,
                LED_PERMANENT,
                LED_PERMANENT,
                LED_PERMANENT,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            };

            char output_report[] = {
                0x52, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x70, 0x00, 0x00, 0x00,
                0xff, 0x27, 0x10, 0x00, 0x32,
                0xff, 0x27, 0x10, 0x00, 0x32,
                0xff, 0x27, 0x10, 0x00, 0x32,
                0xff, 0x27, 0x10, 0x00, 0x32,
                0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            };
            
            
            if (dev->type == SIXAXIS) {
                printf("six\n");
                //set0201[5] = 0xff;  // Enable gyro
                //set0201[6] = 0x78;  // Constant bias (should adjust periodically ?)
            } else {
                printf("ds3\n");
                //set0201[5] = 0;     // Set to e.g. 20 to test rumble on startup
                //set0201[6] = 0x70;  // Weak rumble
            }
            printf("start trans\n");
            //hidp_trans(dev->csk,dev->isk, set0201, sizeof(set0201));
            //hidp_trans(dev->csk,dev->isk, set0201, sizeof(set0201));
            //hidp_trans(dev->csk,dev->isk, output_report, sizeof(output_report));
            printf("finish setup\n");
            break;
        }
        

    }

}

void sixaxis_ds3_parse_report(unsigned char *r, int len) {
    if (r[0] == 0x01 && len >= 49) {
        int aX = r[41]*256 + r[42] - 512;
        int aY = r[43]*256 + r[44] - 512;
        int aZ = r[45]*256 + r[46] - 512;
        int gZ = r[47]*256 + r[48] - 512;
        printf(" aX=%-4d aY=%-4d aZ=%-4d gZ=%-4d", aX, aY, aZ, gZ);
    }
    printf("\n");
}

void parse_report(struct motion_dev *dev, unsigned char *r, int len) {
    struct timeval tv;

    switch (dev->type) {
        case SIXAXIS:
            printf("%d %s SIXAXIS", dev->index, myba2str(&dev->addr));
            sixaxis_ds3_parse_report(r, len);
            break;
        case DS3:
            printf("%d %s DS3    ", dev->index, myba2str(&dev->addr));
            sixaxis_ds3_parse_report(r, len);
            break;
    }
    fflush(stdout);
}

void parse_raw(_sixmotion_hidraw &raw, _sixmotion &sixm) {

    sixm.L3_x = -128.0f + (float)raw.L3_x;
    sixm.L3_y = 128.0f - (float)raw.L3_y;
    sixm.L3_mag = sqrt(sixm.L3_x*sixm.L3_x+sixm.L3_y*sixm.L3_y);
    
    if(sixm.L3_mag > 128.0f)
        sixm.L3_mag = 128.0f;
    
    
    
    
    sixm.R3_x = -128.0f + (float)raw.R3_x;
    sixm.R3_y = 128.0f - (float)raw.R3_y;
    sixm.R3_mag = sqrt(sixm.R3_x*sixm.R3_x+sixm.R3_y*sixm.R3_y);


    if (sixm.R3_mag > 128.0f)
        sixm.R3_mag = 128.0f;

    
    
    uint16_t ax,ay,az,gz;
    
    
    be_memcpy(&ax, raw.ax, 2);
    be_memcpy(&ay, raw.ay, 2);
    be_memcpy(&az, raw.az, 2);
    be_memcpy(&gz, raw.gz, 2);    
    
    sixm.aX = ax -512;
    sixm.aY = ay -512;
    sixm.aZ = az -512-100;
    sixm.gZ = gz -512;    
    //printf("%i \n",ax);
    
    //sixm.aX = raw.ax1*256 + raw.ax2 - 512;
    //sixm.aY = raw.ay1*256 + raw.ay2 - 512;
    //sixm.aZ = raw.az1*256 + raw.az2 - 512;
    //sixm.gZ = raw.gz1*256 + raw.gz2 - 512;
    
    //printf("%f %f %f %f\n",sixm.aX,sixm.aY,sixm.aZ,sixm.gZ);
    
    //sqrt(sixm.aX*sixm.aX+sixm.aY*sixm.aY+sixm.aZ+sixm.aZ);
    
    
    //printf("%f %f %f %f\n",sixm.aX,sixm.aY,sixm.aZ);
    //printf("%f\n",sqrt(sixm.aX*sixm.aX+sixm.aY*sixm.aY+sixm.aZ*sixm.aZ));
    
    //printf("%i %i\n",raw.gz1,raw.gz2);
    
    //printf("%i %i %i %i\n",raw.aX,raw.aY,raw.aZ,raw.gZ);

}