# BitStream (EXPERIMENTAL)

Re-Implementation RakNet::BitStream mainly adapted to work with SA-MP.

**The code has not been tested. Use at your own risk.**
**Functionality tested on client side only.**
==========

**Example to use:**
==========

```cpp
void RPC_Write() {
	IBitStream stream;
	
	int32_t i = 0;
	float f = 3.14f;
	std::string str{"Hello, World!"};
	
	stream << i << f << str;
	
	RakPeer::RPC(RPC_ID, reinterpret_cast<const char*>(stream.GetData()), stream.GetLength(), HIGH_PRIORITY, RELIABLE_ORDERED, 0);
}

void RPC_Read(RPCParameters* params) {
	IBitStream stream(std::move(params));
	
	int i;
	float f;
	std::string str{};
	
	stream >> i >> f >> str;
	
	std::cout << i << f << str << '\n';
}

void RPC_Read2(RPCParameters* params) {
	IBitStream stream(params->input, (params->numberOfBitsOfData + 7) >> 3);
	
	int i;
	float f;
	std::string str{};
	
	stream >> i >> f >> str;
	
	std::cout << i << f << str << '\n';
}
```
**Dependencies:**
==========
Structure from Raknet - RPCParameters

**Functions:**
==========
```cpp
	* operator >> - read BitStream into variable;
	* operator << - write variable or value into BitStream;
	* bool WriteBits(const unsigned char* input, int numberOfBitsToWrite, const bool alignBitsToRight) - write bits into BitStream;
	* inline void Write(const unsigned char* data, int bits) - write bits into BitStream;
	* bool ReadBits(unsigned char* output, int numberOfBitsToRead, const bool alignBitsToRight) - read BitStream bits into variable;
	* inline void Read(unsigned char* data, int bits) - read BitStream bits into variable;
	
	* int GetLength() const - get length of BitStream data size;
	* void IgnoreBits(int bits) - ignore read offset bits (same thing as SetReadOffset(GetReadOffset() + bits));
	* unsigned char* GetData() const - get BitStream data;
	
	* void ResetReadOffset() - set read offset of BitStream to 0U;
	* int GetReadOffset() const - get read offset of BitStream data;
	* void SetReadOffset(int offset) - set read offset of BitStream to value;
	
	* void ResetWriteOffset() - set write offset of BitStream to 0U;
	* int GetWriteOffset() const - get write offset of BitStream data;
	* void SetWriteOffset(int offset) - set write offset of BitStream to value;
```
