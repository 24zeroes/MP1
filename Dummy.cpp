#include <cstdio>
#include "MP1Node.h"

int main(int argc, char *argv[]) {
    
    MessageHdr *msg;
    Address joinaddr;
    memset(&joinaddr, 0, sizeof(Address));
    *(int *)(&joinaddr.addr) = 1;
    *(short *)(&joinaddr.addr[4]) = 0;

    Member *memberNode = new Member;

    size_t msgsize = sizeof(MessageHdr) + sizeof((&joinaddr)->addr) + sizeof(long) + 1;
    msg = (MessageHdr *) malloc(msgsize * sizeof(char));
    
    // create JOINREQ message: format of data is {struct Address myaddr}
    msg->msgType = JOINREQ;
    memcpy((char *)(msg+1), &memberNode->addr.addr, sizeof(memberNode->addr.addr));
    memcpy((char *)(msg+1) + 1 + sizeof(memberNode->addr.addr), &memberNode->heartbeat, sizeof(memberNode->heartbeat));
    printf("hw!");
    
    printf((char *)&msg);
 }  