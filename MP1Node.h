/**********************************
 * FILE NAME: MP1Node.cpp
 *
 * DESCRIPTION: Membership protocol run by this Node.
 * 				Header file of MP1Node class.
 **********************************/

#ifndef _MP1NODE_H_
#define _MP1NODE_H_

#include "stdincludes.h"
#include <cstdint>
#include "Log.h"
#include "Params.h"
#include "Member.h"
#include "EmulNet.h"
#include "Queue.h"

/**
 * Macros
 */
#define T_CLEANUP  10
#define TFAIL 5

/*
 * Note: You can change/add any functions in MP1Node.{h,cpp}
 */

/**
 * Message Types
 */
enum MsgTypes{
    JOINREQ,
    JOINREP,
	HEARTBEAT,
    DUMMYLASTMSGTYPE
};

/**
 * STRUCT NAME: MessageHdr
 *
 * DESCRIPTION: Header and content of a message
 */
typedef struct MessageHdr {
	enum MsgTypes msgType;
}MessageHdr;

typedef struct MemberState {
	long  heartbeat;
	uint   lastHeardTime;
	bool suspected      = false;
	int  suspectedTime  = -1;
    MemberState()
      : heartbeat(0),
        lastHeardTime(0),
        suspected(false),
        suspectedTime(-1)
    {}

    // convenience ctor
    MemberState(long hb, int lastTime, bool isSuspected, int suspectTime)
      : heartbeat(hb),
        lastHeardTime(lastTime),
        suspected(isSuspected),
        suspectedTime(suspectTime)
    {}
}MemberState;

typedef struct AddressCompare {
	bool operator()(const Address &a, Address &b) const {
	  // raw‐byte lexicographical compare
	  return memcmp(a.addr, b.addr, sizeof(a.addr)) < 0;
	}

	bool operator()(const Address &a, const Address &b) const noexcept {
        // lexicographically compare the raw bytes of the addr array
        return memcmp(a.addr, b.addr, sizeof(a.addr)) < 0;
    }
}AddressCompare;

/**
 * CLASS NAME: MP1Node
 *
 * DESCRIPTION: Class implementing Membership protocol functionalities for failure detection
 */
class MP1Node {
private:
	EmulNet *emulNet;
	Log *log;
	Params *par;
	Member *memberNode;
	char NULLADDR[6];
	std::map<Address, MemberState, AddressCompare> membership;
	uint currentTime;

public:
	MP1Node(Member *, Params *, EmulNet *, Log *, Address *);
	Member * getMemberNode() {
		return memberNode;
	}
	int recvLoop();
	static int enqueueWrapper(void *env, char *buff, int size);
	void nodeStart(char *servaddrstr, short serverport);
	int initThisNode(Address *joinaddr);
    int introduceSelfToGroup(Address *joinAddress);
    std::vector<uint8_t> packHEARTBEAT();
    void packJOINREQ(vector<uint8_t> &buf, Address *joinaddr);
    int finishUpThisNode();
	void nodeLoop();
	void checkMessages();
    bool recvCallBack(void *env, char *data, int size);
    std::vector<uint8_t> packJOINREP();
    std::vector<std::pair<Address,long>> unpackJOINREP(const std::vector<uint8_t> &buf);
    std::pair<Address, long> unpackJOINREQ(const std::vector<uint8_t> &buf);
	void logMessage(Address &hbAddress);
    void nodeLoopOps();
    tuple<std::vector<std::pair<Address,long>>, Address> unpackHEARTBEAT(const std::vector<uint8_t> &buf);
    int isNullAddress(Address *addr);
    Address getJoinAddress();
	void initMemberListTable(Member *memberNode);
	void printAddress(Address *addr);
	virtual ~MP1Node();
};

#endif /* _MP1NODE_H_ */
