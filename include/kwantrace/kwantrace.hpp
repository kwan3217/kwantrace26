//
// Created by kwan3217 on 4/14/26.
//
#ifndef KWANTRACE_HPP
#define KWANTRACE_HPP

/***
 * Observation pattern
 * Sometimes one part of the code wants a reference to an object that some other
 * part of the code owns (IE is responsible for cleanup for). We call this an "observer",
 * both the code that is observing the foreign object, and the mechanism which does
 * the referencing. In C this would be a pointer. In C++ we would want it to be a
 * reference but references can't be changed after they are created, and must always
 * reference *something*.
 *
 * A former version of this library used the following code:
 * ```
   // Alias for a pointer of a given type, intended to indicate intent that
   // this pointer does not own anything and should never be used to delete, free,
   // or otherwise deallocate what is being pointed at.
   template<typename T>
   using Observer= const T*;
   ```
 * This gives a name to a pointer to an object which can be observed, with the implication
 * of non-ownership. Also, an observer is *read-only* -- it can't change fields or call
 * non-const methods.
 *
 * An LLM has convinced me that the current idiomatic way to do this in modern C++ is to just
 * use the bare pointers or references:
 *
 * `const T*` observer pointer to object, might be null
 * `const T&` observer reference, must reference something
 * `T*` observer-and-mutator pointer to an object, might be null
 * `T&` observer-and-mutator reference to an object, must reference something
 */

#include "math.hpp"
#include "pdvector.hpp"
#include "transformation.hpp"
#include "transformable.hpp"
#include "renderable.hpp"

#endif // KWANTRACE_HPP
