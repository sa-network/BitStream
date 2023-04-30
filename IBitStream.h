/**
 * @file IBitStream.h
 * @brief Header file for SA Network
 *
 * This file is part of SA Network.
 *
 * SA Network is free software: you can redistribute it and/or modify
 * it under the terms of the Apache License 2.0.
 *
 * SA Network is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License 2.0 for more details.
 *
 * You should have received a copy of the Apache License 2.0
 * along with SA Network. If not, see <http://www.apache.org/licenses/>.
 */
#ifndef I_BIT_STREAM_H
#define I_BIT_STREAM_H

#include <algorithm>
#include <memory>

#ifndef __NETWORK_TYPES_H
#include <NetworkTypes.h> //RakNet include
#endif

class IBitStream {
public:
    IBitStream() 
        : length_(0), data_(), read_offset_(0), write_offset_(0) 
    {}

    explicit IBitStream(unsigned char* data, int length)
        : length_(length), data_(data), read_offset_(0), write_offset_(0)
    {}

    explicit IBitStream(const RPCParameters* params) 
        : IBitStream(params->input, (params->numberOfBitsOfData + 7) >> 3)
    {}

    template<typename U>
    static constexpr int BITS_IN_T = sizeof(U) << 3;

    template<typename T>
    IBitStream& operator<<(const T value) {
        if constexpr (std::is_same_v<T, bool>) {
            this->Reserve(1);
            data_[write_offset_ >> 3] |= (value << (7 - (write_offset_ % 8)));
            write_offset_++;
        }
        else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double> || std::is_same_v<T, unsigned char> ||
            std::is_same_v<T, unsigned short> || std::is_same_v<T, unsigned int> || std::is_same_v<T, unsigned long> ||
            std::is_same_v<T, unsigned long long>) {
            Reserve(BITS_IN_T<T>);
            WriteBits(reinterpret_cast<const unsigned char*>(&value), BITS_IN_T<T>, true);
        }
        else if constexpr (std::is_same_v<T, std::string>) {
            constexpr auto size = BITS_IN_T<T>;
            Reserve(size);
            WriteBits(reinterpret_cast<const unsigned char*>(&value[0]), size, true);
        }
        else if constexpr (std::is_same_v<T, int8_t> || std::is_same_v<T, int16_t> || std::is_same_v<T, int32_t> || std::is_same_v<T, int64_t>) {
            Reserve(BITS_IN_T<T>);
            Write(reinterpret_cast<const unsigned char*>(&value), BITS_IN_T<T>);
        }
        else {
            static_assert(std::is_same_v<T, T>, "Unsupported type");
        }
        return *this;
    }

    bool WriteBits(const unsigned char* input, int numberOfBitsToWrite, const bool alignBitsToRight) {
        if (numberOfBitsToWrite < 1)
            return false;

        int writeOffsetMod8 = write_offset_ & 7;
        const unsigned char* pInput = input;

        while (numberOfBitsToWrite > 0) {
            data_[write_offset_ >> 3] |= *(pInput++) >> (alignBitsToRight ? (8 - writeOffsetMod8 - numberOfBitsToWrite) : (writeOffsetMod8)) & ((numberOfBitsToWrite > 8 - writeOffsetMod8) ? 0xFF >> writeOffsetMod8 : (0xFF >> writeOffsetMod8) & (0xFF << (8 - writeOffsetMod8 - numberOfBitsToWrite)));
            numberOfBitsToWrite -= 8;

            write_offset_ += (numberOfBitsToWrite < 0) ? (8 + numberOfBitsToWrite) : 8;

            if (numberOfBitsToWrite < 0) {
                data_[write_offset_ / 8 - 1] = (data_[write_offset_ / 8 - 1] >> (-numberOfBitsToWrite)) << (-numberOfBitsToWrite);
                return true;
            }
        }
        return true;
    }

    inline void Write(const unsigned char* data, int bits) {
        const int bytes = (bits + 7) >> 3;
        const uint32_t* data32 = reinterpret_cast<const uint32_t*>(data);
        uint32_t* dest32 = reinterpret_cast<uint32_t*>(data_ + (write_offset_ >> 3));
        const int num_words = bytes >> 2;
        const int num_bytes = bytes & 3;
        const int bit_offset = write_offset_ & 7;
        const uint32_t mask = 0xffffffffu >> bit_offset;

        for (int i = 0; i < num_words; ++i) {
            const uint32_t value = *data32++;
            *dest32 = (*dest32 & mask) | (value << bit_offset);
            ++dest32;
        }

        if (num_bytes > 0) {
            uint32_t value = 0;
            memcpy(&value, data32, num_bytes);
            value = (value << bit_offset) | (*dest32 & ~mask);
            *dest32 = value;
        }
        write_offset_ += bits;
    }

    template<typename T>
    IBitStream& operator>>(T& value) {
        if constexpr (std::is_same_v<T, bool>) {
            value = (data_[read_offset_ >> 3] & (0x80 >> (read_offset_ % 8))) != 0;
            read_offset_++;
        }
        else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double> || std::is_same_v<T, unsigned char> ||
            std::is_same_v<T, unsigned short> || std::is_same_v<T, unsigned int> || std::is_same_v<T, unsigned long> ||
            std::is_same_v<T, unsigned long long>) {
            ReadBits(reinterpret_cast<unsigned char*>(&value), BITS_IN_T<T>, true);
        }
        else if constexpr (std::is_same_v<T, std::string>) {
            constexpr auto size = BITS_IN_T<T>;
            ReadBits(reinterpret_cast<unsigned char*>(&value[0]), size, true);
            value[size] = '\0';
        }
        else if constexpr (std::is_same_v<T, int8_t> || std::is_same_v<T, int16_t> || std::is_same_v<T, int32_t> || std::is_same_v<T, int64_t>) {
            T temp;
            Read(reinterpret_cast<unsigned char*>(&temp), BITS_IN_T<T>);
            value = static_cast<T>(temp);
            return *this;
        }
        else {
            static_assert(std::is_same_v<T, T>, "Unsupported type");
        }
        return *this;
    }

    bool ReadBits(unsigned char* output, int numberOfBitsToRead, const bool alignBitsToRight) {
        if (numberOfBitsToRead <= 0)
            return false;

        int readOffsetMod8 = read_offset_ & 7;
        unsigned char* pOutput = output;

        while (numberOfBitsToRead > 0) {
            *(pOutput++) |= *(data_ + (read_offset_ >> 3)) << (readOffsetMod8 & 7);
            *(pOutput - 1) |= (*(data_ + (read_offset_ >> 3) + 1) >> (8 - (readOffsetMod8))) & ((numberOfBitsToRead > 8 - (readOffsetMod8)) ? 0xFF : 0);
            numberOfBitsToRead -= 8;

            read_offset_ += (numberOfBitsToRead < 0) ? (8 + numberOfBitsToRead) : 8;

            if (numberOfBitsToRead < 0) {
                *(pOutput - 1) = (*(pOutput - 1) << (-numberOfBitsToRead)) & ((1 << (-numberOfBitsToRead)) - 1);
                *(pOutput - 1) >>= (alignBitsToRight) ? -numberOfBitsToRead : 0;
                return true;
            }
        }
        return true;
    }

    inline void Read(unsigned char* data, int bits) {
        const int bytes = (bits + 7) >> 3;
        const uint32_t* src32 = reinterpret_cast<const uint32_t*>(data_ + (read_offset_ >> 3));
        uint32_t* data32 = reinterpret_cast<uint32_t*>(data);
        const int num_words = bytes >> 2;
        const int num_bytes = bytes & 3;
        const int bit_offset = read_offset_ & 7;
        const uint32_t mask = 0xffffffffu << bit_offset;

        for (int i = 0; i < num_words; ++i) {
            const uint32_t value = *src32++;
            *data32++ = (value >> bit_offset) | (*(src32 - 1) & mask);
        }

        if (num_bytes > 0) {
            uint32_t value = *src32;
            value = (value & mask) >> bit_offset;
            memcpy(data32, &value, num_bytes);
        }
        read_offset_ += bits;
    }

    int GetLength() const {
        return length_;
    }

    void IgnoreBits(int bits) {
        read_offset_ += bits;
    }

    unsigned char* GetData() const {
        return data_;
    }

    void ResetReadOffset() {
        read_offset_ = 0;
    }

    int GetReadOffset() const {
        return read_offset_;
    }

    void SetReadOffset(int offset) {
        read_offset_ = offset;
    }

    void ResetWriteOffset() {
        write_offset_ = 0;
    }

    int GetWriteOffset() const {
        return write_offset_;
    }

    void SetWriteOffset(int offset) {
        write_offset_ = offset;
    }

private:
    void Reserve(int bits) {
        if (bits < 1)
            return;

        constexpr int BITS_IN_BYTE = 8;
        const auto byte_offset = write_offset_ >> 3;
        const auto required_bytes = ((bits + BITS_IN_BYTE + 7) >> 3);

        if (byte_offset + required_bytes <= length_)
            return;

        const auto new_length = byte_offset + required_bytes;
        auto new_data = std::make_unique<unsigned char[]>(new_length);
        if (data_ && new_data.get() != data_)
            std::memmove(new_data.get(), data_, length_);
        data_ = new_data.release();
        length_ = new_length;
    }

    int length_;
    unsigned char* data_;
    int read_offset_;
    int write_offset_;
};

#endif //I_BIT_STREAM