#ifndef SWAP_ENDIAN_H_
#define SWAP_ENDIAN_H_

#include <algorithm>
#include <array>

template<class T>
T swap_endian(T value) {
  // static_assert(std::has_unique_object_representations_v<T>, "T may have padding bits");
  unsigned char* first = reinterpret_cast<unsigned char*>(&value);
  unsigned char* last = first + sizeof(T);
  std::reverse(first, last);
  return value;
}
template<class T, int N>
std::array<T, N> swap_endian(const T (&arr)[N]) {
  std::array<T, N> ret = {};
  for(int i = 0; i < N; ++i) {
    ret[i] = swap_endian(arr[i]);
  }
  return ret;
}
template<class T, int N>
std::array<T, N> swap_endian(const std::array<T, N>& arr) {
  return swap_endian(reinterpret_cast<const T(&)[N]>(arr.data()));
}

#endif  // SWAP_ENDIAN_H_
