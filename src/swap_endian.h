#ifndef SWAP_ENDIAN_H_
#define SWAP_ENDIAN_H_

#include <algorithm>

template<class T>
T swap_endian(T value) {
  // static_assert(std::has_unique_object_representations_v<T>, "T may have padding bits");
  unsigned char* first = reinterpret_cast<unsigned char*>(&value);
  unsigned char* last = first + sizeof(T);
  std::reverse(first, last);
  return value;
}

#endif  // SWAP_ENDIAN_H_
