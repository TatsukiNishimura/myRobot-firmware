#pragma once

#include <vector>
#include <cstdint>

using std::vector;

/**
 * @brief シリアル化
 * @param args シリアル化対象の変数
 * @return シリアル化されたvector
 *
 * @note シリアル化可能な値：プリミティブ型とそのstd::array
 * （おそらくmemcpyとsizeofが適切にできる型ならok）
 * @note 異なるプラットフォームに送るときはuint16_t, int32_t等のバイト数が決まっている型を使用した方が安全。バイトオーダーに注意。
 *
 * 以下のように使用する
 * @code
 * vector<uint8_t> serialized=serialize(uint32_var, array_int16_var, float_var);
 * @endcode
 **/
template <class... Args>
vector<uint8_t> serialize(Args... args);

/**
 * @brief デシリアライズ
 * @param serialized シリアル化されたvector
 * @param args 値を取りだすときの変数。シリアライズのときに使用した引数と同じ型の変数を、同じ順番に並べる。
 * @return デシリアライズが成功したら真、そうでなければ偽を返す
 *
 *
 * 以下のように使用する。
 * @code
 * deserialize(des_uint32_var, des_array_int16_var, des_float_var);
 * @endcode
 **/
template <class... Args>
bool deserialize(vector<uint8_t> &serialized, Args &...args);

//-------------- 実装 ----------------//

#include <string.h>

constexpr size_t byteSize()
{
    return 0;
}

/**
 * @brief 引数のバイトサイズの合計を求める
 * @return 引数のバイトサイズの合計
 *
 * @code
 * size_t size=byteSize(uint32_var, array_int16_var, float_var)
 * @endcode
 **/
template <class tFirst, class... tRest>
constexpr size_t byteSize(tFirst first, tRest... rest)
{
    return (sizeof...(rest)) ? sizeof(first) + byteSize(rest...) : sizeof(first);
}

template <class tFirst>
void inter_serialize(size_t pos, vector<uint8_t> &serialized, tFirst first)
{
    memcpy(&serialized[pos], &first, sizeof(first));
}

template <class tFirst, class... tRest>
void inter_serialize(size_t pos, vector<uint8_t> &serialized, tFirst first, tRest... rest)
{
    memcpy(&(serialized[pos]), &first, sizeof(first));
    inter_serialize(pos + sizeof(first), serialized, rest...);
}

template <class... Args>
vector<uint8_t> serialize(Args... args)
{
    vector<uint8_t> serialized(byteSize(args...));
    inter_serialize(0, serialized, args...);
    return serialized;
}

template <class tFirst>
void inter_deserialize(size_t pos, vector<uint8_t> &serialized, tFirst &first)
{
    memcpy(&first, &serialized[pos], sizeof(first));
}

template <class tFirst, class... tRest>
void inter_deserialize(size_t pos, vector<uint8_t> &serialized, tFirst &first, tRest &...rest)
{
    memcpy(&first, &(serialized[pos]), sizeof(first));
    inter_deserialize(pos + sizeof(first), serialized, rest...);
}

template <class... Args>
bool deserialize(vector<uint8_t> &serialized, Args &...args)
{
    if (serialized.size() != byteSize(args...))
    {
        return false;
    }
    inter_deserialize(0, serialized, args...);

    return true;
}
