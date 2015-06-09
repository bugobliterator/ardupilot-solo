// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           École Polytechnique de Montréal
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef KVECTOR_IMPL_HPP
#define KVECTOR_IMPL_HPP

//! \file
//! \brief Contains the implementation of the \c KVector template class.

namespace Kalman {
  //! Refers to the currently selected vector printing context.

  //! \warning Never modify this value directly. Use 
  //! <tt>selectKVectorContext()</tt> instead.

  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector()
    : v_(0), n_(0) {}

  //! \param n Number of elements in vector. Can be 0.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector(K_UINT_32 n)
    : vimpl_(n), v_( (n != 0) ? &vimpl_[0] - BEG : 0 ), n_(n) {}

  //! \param n Number of elements in vector. Can be 0.
  //! \param a Value to copy multiple times in the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector(K_UINT_32 n, const T& a)
    : vimpl_(n, a), v_( (n != 0) ? &vimpl_[0] - BEG : 0 ), n_(n) {}

  //! This function allows to transform a C-style array of \c T objects in
  //! a <tt>KVector<T, BEG, DBG></tt> equivalent array. Note that objects from 
  //! the C-style array are copied into the vector, which may slow down
  //! application if used extensively.
  //! \param n Size of the \c v array. Can be 0.
  //! \param v Pointer to an \c n-size array of \c T objects.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector(K_UINT_32 n, const T* v)
    : vimpl_(v, v + n), v_( (n != 0) ? &vimpl_[0] - BEG : 0 ), n_(n) {}

  //! \param v Vector to copy. Can be an empty vector.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::KVector(const KVector& v) 
    : vimpl_(v.vimpl_), v_( (v.size() != 0) ? &vimpl_[0] - BEG : 0 ), 
      n_(v.n_) {}

  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>::~KVector() {}

  //! \param i Index of element to retrieve from vector.
  //! \return A reference to the \c i'th element. 
  //! \exception OutOfBoundError Thrown if \c i is out of vector bounds and 
  //!   <tt>DBG == true</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline T& KVector<T, BEG, DBG>::operator()(K_UINT_32 i) {
    return v_[i];
  }

  //! \param i Index of element to retrieve from vector.
  //! \return A \c const reference to the \c i'th element. 
  //! \exception OutOfBoundError Thrown if i is out of vector bounds and 
  //!   <tt>DBG == true</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline const T& KVector<T, BEG, DBG>::operator()(K_UINT_32 i) const {
    return v_[i];
  }

  //! \return The number of elements in the vector.
  //!
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline K_UINT_32 KVector<T, BEG, DBG>::size() const {
    return n_;
  }

  //! \param n The new size of the vector. Can be 0.
  //! \warning This function may invalidates pointers inside the vector.
  //! \note Resizing to a smaller size does not free any memory. To do so,
  //! one can swap the vector to shrink with a temporary copy of itself :
  //! <tt>KVector<T, BEG, DBG>(v).swap(v)</tt>.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KVector<T, BEG, DBG>::resize(K_UINT_32 n) {
    
    if (n == n_) {
      return;
    }
    
    vimpl_.resize(n);
    v_ = (n != 0) ? &vimpl_[0] - BEG : 0;
    n_ = n;
  }

  //! \param a Instance to copy to each element of vector.
  //! \return A reference to the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>& KVector<T, BEG, DBG>::operator=(const T& a) {
    if (n_ == 0) {
      return *this;
    }

    T* ptr = &vimpl_[0];
    const T* end = ptr + n_;

    while (ptr != end) {
      *ptr++ = a;
    }
    return *this;
  }

  //! \param v Vector to copy.
  //! \return A reference to the assigned vector.
  //! \warning This function invalidates pointers inside the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline KVector<T, BEG, DBG>& 
  KVector<T, BEG, DBG>::operator=(const KVector& v) {
    KVector temp(v);
    swap(temp);
    return *this;
  }

  //! \param n Size of C-style array.
  //! \param v Pointer to first element to copy from C-style array.
  //! \warning This function invalidates pointers inside the vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KVector<T, BEG, DBG>::assign(K_UINT_32 n, const T* v) {
    KVector temp(n, v);
    swap(temp);
  }

  //! This function is fast, since it exchanges pointers to underlying
  //! implementation without copying any element.
  //! \param v Vector to swap.
  //! \note No pointer is invalidated by this function. They still point to
  //!   the same element, but in the other vector.
  template<typename T, K_UINT_32 BEG, bool DBG>
  inline void KVector<T, BEG, DBG>::swap(KVector& v) {
    vimpl_.swap(v.vimpl_);
    Util::swap(v_, v.v_);
    Util::swap(n_, v.n_);
  }
}

#endif
