#include "MP1Node.h"

MP1Node::MP1Node(Member *member, Params *params, EmulNet *emul, Log *log, Address *address) {
	for( int i = 0; i < 6; i++ ) {
		NULLADDR[i] = 0;
	}
	this->memberNode = member;
	this->emulNet = emul;
	this->log = log;
	this->par = params;
	this->memberNode->addr = *address;
    this->currentTime = params->getcurrtime();
}

MP1Node::~MP1Node() {}

int MP1Node::recvLoop() {
    if ( memberNode->bFailed ) {
    	return false;
    }
    else {
    	return emulNet->ENrecv(&(memberNode->addr), enqueueWrapper, NULL, 1, &(memberNode->mp1q));
    }
}

int MP1Node::enqueueWrapper(void *env, char *buff, int size) {
	Queue q;
	return q.enqueue((queue<q_elt> *)env, (void *)buff, size);
}

void MP1Node::nodeStart(char *servaddrstr, short servport) {
    Address joinaddr;
    joinaddr = getJoinAddress();

    // Self booting routines
    if (initThisNode(&joinaddr) == -1) {
        exit(1);
    }

    if (!introduceSelfToGroup(&joinaddr)) {
        finishUpThisNode();
        exit(1);
    }
    return;
}

int MP1Node::initThisNode(Address *joinaddr) {
	int id = *(int*)(&memberNode->addr.addr);
	int port = *(short*)(&memberNode->addr.addr[4]);

	memberNode->bFailed = false;
	memberNode->inited = true;
	memberNode->inGroup = false;
    // node is up!
	memberNode->nnb = 0;
	memberNode->heartbeat = 0;
	memberNode->pingCounter = TFAIL;
	memberNode->timeOutCounter = -1;
    initMemberListTable(memberNode);

    return 0;
}

int MP1Node::introduceSelfToGroup(Address *joinaddr) {
    if ( 0 == memcmp((char *)&(memberNode->addr.addr), (char *)&(joinaddr->addr), sizeof(memberNode->addr.addr))) {
        membership[memberNode->addr] = { memberNode->heartbeat, currentTime, false, -1 };
        log->logNodeAdd(&memberNode->addr, &memberNode->addr);
        memberNode->inGroup = true;
    }
    else {
        std::vector<uint8_t> buf;
        packJOINREQ(buf, joinaddr);
        emulNet->ENsend(&memberNode->addr, joinaddr, 
            reinterpret_cast<char*>((&buf)->data()), int((&buf)->size()));
    }

    return 1;
}

std::vector<std::pair<Address,long>>
MP1Node::unpackHEARTBEAT(const std::vector<uint8_t> &buf) {
    // (very similar to unpackJOINREP but no msgType check for JOINREP)
    auto *hdr = reinterpret_cast<const MessageHdr*>(buf.data());
    if (hdr->msgType != HEARTBEAT) throw std::runtime_error("not HB");
    uint32_t payloadLen = *reinterpret_cast<const uint32_t*>(
                             buf.data() + sizeof(MessageHdr));
    const uint8_t *p = buf.data() + sizeof(MessageHdr) + sizeof(uint32_t);

    uint32_t N; memcpy(&N, p, sizeof(N)); p += sizeof(N);
    constexpr size_t A = sizeof(memberNode->addr.addr);
    constexpr size_t H = sizeof(memberNode->heartbeat);
    std::vector<std::pair<Address,long>> out;
    out.reserve(N);

    for (uint32_t i = 0; i < N; i++) {
      Address a; long h;
      memcpy(a.addr, p, A); p += A;
      memcpy(&h,    p, H); p += H;
      out.emplace_back(a,h);
    }
    return out;
}

std::vector<uint8_t> MP1Node::packHEARTBEAT() {
    uint32_t N = membership.size();
    constexpr size_t A = sizeof(memberNode->addr.addr);
    constexpr size_t H = sizeof(memberNode->heartbeat);
    uint32_t payloadLen = sizeof(uint32_t) + N*(A+H);

    std::vector<uint8_t> buf(sizeof(MessageHdr) + sizeof(uint32_t) + payloadLen);
    auto *hdr = reinterpret_cast<MessageHdr*>(buf.data());
    hdr->msgType = HEARTBEAT;

    // size field
    *reinterpret_cast<uint32_t*>(buf.data() + sizeof(MessageHdr))
      = payloadLen;

    // payload
    uint8_t *p = buf.data() + sizeof(MessageHdr) + sizeof(uint32_t);
    memcpy(p, &N, sizeof(N));  p += sizeof(N);

    for (auto &kv : membership) {
      memcpy(p, kv.first.addr, A);  p += A;
      memcpy(p, &kv.second.heartbeat, H);  p += H;
    }
    return buf;
}

void MP1Node::packJOINREQ(std::vector<uint8_t> &buf, Address *joinaddr)
{
    size_t payloadLen = sizeof(MessageHdr) + sizeof(joinaddr->addr) + sizeof(long) + 1;
    buf.resize(sizeof(MessageHdr) + sizeof(uint32_t) + payloadLen);

    // 1) header lives at buf.data()
    auto * hdrPtr = reinterpret_cast<MessageHdr*>(buf.data());
    hdrPtr->msgType = JOINREQ;
    // etc...

    // 2) size field
    auto * sizePtr = reinterpret_cast<uint32_t*>(buf.data() + sizeof(MessageHdr));
    *sizePtr = uint32_t(payloadLen);

    // 3) payload region
    uint8_t * dataPtr = buf.data() + sizeof(MessageHdr) + sizeof(uint32_t);
    memcpy(dataPtr, &memberNode->addr.addr, sizeof(memberNode->addr.addr));
    memcpy(dataPtr + sizeof(memberNode->addr.addr), &memberNode->heartbeat, sizeof(&memberNode->heartbeat));
}

int MP1Node::finishUpThisNode(){
   return 0;
}

void MP1Node::nodeLoop() {
    if (memberNode->bFailed) {
    	return;
    }
    checkMessages();
    if( !memberNode->inGroup ) {
    	return;
    }
    nodeLoopOps();
    return;
}

void MP1Node::checkMessages() {
    void *ptr;
    int size;
    while ( !memberNode->mp1q.empty() ) {
    	ptr = memberNode->mp1q.front().elt;
    	size = memberNode->mp1q.front().size;
    	memberNode->mp1q.pop();
    	recvCallBack((void *)memberNode, (char *)ptr, size);
    }
    return;
}

bool MP1Node::recvCallBack(void *env, char *data, int size) {
    std::vector<uint8_t> buf(data, data + size);
    auto * hdr = reinterpret_cast<const MessageHdr*>(buf.data());
    switch (hdr->msgType) {
      case JOINREQ:
      {
        Address peerAddr;
        long    peerHB;
        try {
          std::tie(peerAddr, peerHB) = unpackJOINREQ(buf);
          logMessage(hdr, peerHB, peerAddr);
        } catch (const std::exception &e) {
          // malformed packet
          return false;
        }
        if (membership.find(peerAddr) == membership.end()) {
            log->logNodeAdd(&memberNode->addr, &peerAddr);
        }
        membership[peerAddr] = { peerHB, currentTime, false, -1 };
        auto replyBuf = packJOINREP();
        emulNet->ENsend(&memberNode->addr,
                        &peerAddr,
                        reinterpret_cast<char*>(replyBuf.data()),
                        static_cast<int>(replyBuf.size()));
        break;
      }

      case JOINREP: {
        memberNode->inGroup = true;
        std::vector<std::pair<Address,long>> members;
        try {
          members = unpackJOINREP(buf);

        } catch(...) {
          return false;
        }
        for (auto &pr : members) {
          Address &addr = pr.first;
          long hb = pr.second;
          if (membership.find(addr) == membership.end()) {
            log->logNodeAdd(&memberNode->addr, &addr);
          }
          membership[addr] = { hb, currentTime, false, -1 };
        }
        break;
      }

      case HEARTBEAT: {
        // wrap raw buffer
        std::vector<uint8_t> buf(data, data + size);
        auto entries = unpackHEARTBEAT(buf);
    
        for (auto &pr : entries) {
          const Address &addr = pr.first;
          long            hb   = pr.second;
          auto  it = membership.find(addr);
    
          if (it == membership.end()) {
            // new node you’ve never seen
            log->logNodeAdd(&memberNode->addr, &const_cast<Address&>(addr));
            membership[addr] = MemberState { hb, currentTime, false, -1 };
          } else {
            // existing node—update if newer
            MemberState &st = it->second;
            if (hb > st.heartbeat) {
              st.heartbeat     = hb;
              st.lastHeardTime = currentTime;
            }
          }
        }
        break;
    }
      default:
        return false;
    }
    return true;
}

std::vector<uint8_t> MP1Node::packJOINREP() {
    // 1) figure out how many members we have
    uint32_t numMembers = static_cast<uint32_t>(membership.size());

    // 2) each entry is (Address.addr + long heartbeat)
    constexpr size_t ADDR_SIZE = sizeof(memberNode->addr.addr);
    constexpr size_t HB_SIZE   = sizeof(memberNode->heartbeat);
    constexpr size_t ENTRY_SZ  = ADDR_SIZE + HB_SIZE;

    // 3) payload = [ count:uint32_t ] + numMembers * ENTRY_SZ
    uint32_t payloadLen = sizeof(uint32_t) + numMembers * ENTRY_SZ;

    // 4) total buffer = header + size‐field + payload
    std::vector<uint8_t> buf;
    buf.resize(sizeof(MessageHdr)
             + sizeof(uint32_t)
             + payloadLen);

    // 5) write the header
    auto *hdr = reinterpret_cast<MessageHdr*>(buf.data());
    hdr->msgType = JOINREP;

    // 6) write the payload length
    auto *szPtr = reinterpret_cast<uint32_t*>(
        buf.data() + sizeof(MessageHdr));
    *szPtr = payloadLen;

    // 7) write the payload
    uint8_t *p = buf.data()
               + sizeof(MessageHdr)
               + sizeof(uint32_t);

    // 7a) write count
    memcpy(p, &numMembers, sizeof(numMembers));
    p += sizeof(numMembers);

    // 7b) write each (addr, heartbeat)
    for (auto &kv : membership) {
        // Address bytes
        memcpy(p,
                    kv.first.addr,
                    ADDR_SIZE);
        p += ADDR_SIZE;

        // heartbeat
        memcpy(p,
                    &kv.second.heartbeat,
                    HB_SIZE);
        p += HB_SIZE;
    }

    return buf;
}

std::vector<std::pair<Address,long>>
MP1Node::unpackJOINREP(const std::vector<uint8_t> &buf) {
    // minimal size = header + size‐field + count
    if (buf.size() < sizeof(MessageHdr) + sizeof(uint32_t) + sizeof(uint32_t)) {
        throw std::runtime_error("unpackJOINREP: buffer too small");
    }

    // 1) header check
    auto *hdr = reinterpret_cast<const MessageHdr*>(buf.data());
    if (hdr->msgType != JOINREP) {
        throw std::runtime_error("unpackJOINREP: wrong msgType");
    }

    // 2) payload length
    auto *szPtr = reinterpret_cast<const uint32_t*>(
        buf.data() + sizeof(MessageHdr));
    uint32_t payloadLen = *szPtr;

    // 3) verify the buffer really contains that many bytes
    size_t totalNeeded = sizeof(MessageHdr)
                       + sizeof(uint32_t)
                       + payloadLen;
    if (buf.size() < totalNeeded) {
        throw std::runtime_error("unpackJOINREP: truncated payload");
    }

    // 4) start of payload
    const uint8_t *p = buf.data()
                     + sizeof(MessageHdr)
                     + sizeof(uint32_t);

    // 5) read count
    uint32_t numMembers;
    memcpy(&numMembers, p, sizeof(numMembers));
    p += sizeof(numMembers);

    // 6) each entry size
    constexpr size_t ADDR_SIZE = sizeof(memberNode->addr.addr);
    constexpr size_t HB_SIZE   = sizeof(memberNode->heartbeat);
    constexpr size_t ENTRY_SZ  = ADDR_SIZE + HB_SIZE;

    // sanity: payloadLen should match count*ENTRY_SZ + sizeof(count)
    if (payloadLen != sizeof(numMembers) + numMembers * ENTRY_SZ) {
        throw std::runtime_error("unpackJOINREP: bad payloadLen");
    }

    // 7) extract all entries
    std::vector<std::pair<Address,long>> members;
    members.reserve(numMembers);
    for (uint32_t i = 0; i < numMembers; ++i) {
        Address addr;
        long    hb;

        memcpy(addr.addr, p, ADDR_SIZE);
        p += ADDR_SIZE;

        memcpy(&hb,    p, HB_SIZE);
        p += HB_SIZE;

        members.emplace_back(addr, hb);
    }

    return members;
}

std::pair<Address,long> 
MP1Node::unpackJOINREQ(const std::vector<uint8_t> &buf) {
    // 1) pointer to the header
    if(buf.size() < sizeof(MessageHdr) + sizeof(uint32_t)) {
        throw std::runtime_error("buffer too small for JOINREQ");
    }
    auto *hdrPtr = reinterpret_cast<const MessageHdr*>(buf.data());
    if(hdrPtr->msgType != JOINREQ) {
        throw std::runtime_error("unpackJOINREQ: wrong msgType");
    }

    // 2) read the payload length
    auto *sizePtr = reinterpret_cast<const uint32_t*>(
        buf.data() + sizeof(MessageHdr));
    uint32_t payloadLen = *sizePtr;
    size_t expectedTotal = sizeof(MessageHdr)
                         + sizeof(uint32_t)
                         + payloadLen;
    if(buf.size() < expectedTotal) {
        throw std::runtime_error("buffer shorter than payloadLen");
    }

    // 3) locate the payload
    const uint8_t *dataPtr = buf.data()
                           + sizeof(MessageHdr)
                           + sizeof(uint32_t);

    // 4) copy out the Address
    Address addr;
    static_assert(sizeof(addr.addr) == 6, 
                  "check your Address.addr size");
    memcpy(addr.addr,
                dataPtr,
                sizeof(addr.addr));
    
    // 5) copy out the heartbeat
    long heartbeat;
    memcpy(&heartbeat,
                dataPtr + sizeof(addr.addr),
                sizeof(heartbeat));

    return { addr, heartbeat };
}

void MP1Node::logMessage(const MessageHdr *hdr, int64_t hb, Address &addr)
{
    #ifdef DEBUGLOG
    static char s[1024];
    snprintf(s, sizeof s,
             "RECIEVED type: %d hb: %d from %u.%u.%u.%u:%u\n",
             hdr->msgType,
             hb,
             (unsigned)addr.addr[0],
             (unsigned)addr.addr[1],
             (unsigned)addr.addr[2],
             (unsigned)addr.addr[3],
             (unsigned)*(uint16_t *)(addr.addr + 4));

    log->LOG(&memberNode->addr, s);
    #endif
}

void MP1Node::nodeLoopOps() {
    ++currentTime;
    memberNode->heartbeat++;
    membership[memberNode->addr].heartbeat     = memberNode->heartbeat;
    membership[memberNode->addr].lastHeardTime = currentTime;

    std::vector<uint8_t> buf;
    {
      size_t payloadLen = sizeof(MessageHdr)
                        + sizeof(memberNode->addr.addr)
                        + sizeof(memberNode->heartbeat);
      buf.resize(sizeof(MessageHdr) + sizeof(uint32_t) + payloadLen);
      auto *hdr = reinterpret_cast<MessageHdr*>(buf.data());
      hdr->msgType = HEARTBEAT;
      *reinterpret_cast<uint32_t*>(buf.data() + sizeof(MessageHdr))
        = uint32_t(payloadLen);
      uint8_t *p = buf.data() + sizeof(MessageHdr) + sizeof(uint32_t);
      memcpy(p,
             &memberNode->addr.addr,
             sizeof(memberNode->addr.addr));
      memcpy(p + sizeof(memberNode->addr.addr),
             &memberNode->heartbeat,
             sizeof(memberNode->heartbeat));
    }
    auto hbBuf = packHEARTBEAT();
    char * data = reinterpret_cast<char*>(hbBuf.data());
    int    len  = static_cast<int>(hbBuf.size());
    
    for (auto const &kv : membership) {
        const Address &destKey = kv.first;
        if (memcmp(destKey.addr, memberNode->addr.addr, sizeof destKey.addr)==0)
          continue;
        Address tmp = destKey;
        emulNet->ENsend(&memberNode->addr, &tmp, data, len);
    }

    std::vector<Address> toRemove;
    std::vector<Address> toSuspect;

    for (auto &kv : membership) {
      const Address &addr = kv.first;
      MemberState &st = kv.second;
      if (memcmp(addr.addr,
        memberNode->addr.addr,
        sizeof(addr.addr)) == 0) {
            continue;
      }

      int silent = currentTime - st.lastHeardTime;
      if (!st.suspected && silent > TFAIL) {
        st.suspected     = true;
        st.suspectedTime = currentTime;
        toSuspect.push_back(addr);
    
      } else if (st.suspected && (currentTime - st.suspectedTime > T_CLEANUP)) {
        toRemove.push_back(addr);
      }
    }

    // 6) Log and erase timed-out members
    for (auto &bad : toSuspect) {
        log->logNodeRemove(&memberNode->addr, &bad);
        membership.erase(bad);
      }
      
      // 3) Finally, erase them from your map after T_CLEANUP
      for (auto &bad : toRemove) {
        log->logNodeRemove(&memberNode->addr, &bad);
        membership.erase(bad);
    }
}

int MP1Node::isNullAddress(Address *addr) {
	return (memcmp(addr->addr, NULLADDR, 6) == 0 ? 1 : 0);
}

Address MP1Node::getJoinAddress() {
    Address joinaddr;

    memset(&joinaddr, 0, sizeof(Address));
    *(int *)(&joinaddr.addr) = 1;
    *(short *)(&joinaddr.addr[4]) = 0;

    return joinaddr;
}

void MP1Node::initMemberListTable(Member *memberNode) {
	memberNode->memberList.clear();
}

void MP1Node::printAddress(Address *addr)
{
    printf("%d.%d.%d.%d:%d \n",  addr->addr[0],addr->addr[1],addr->addr[2],
                                                       addr->addr[3], *(short*)&addr->addr[4]) ;    
}
