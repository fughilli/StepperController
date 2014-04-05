/*
 * simple_math.h
 *
 *  Created on: Jul 20, 2012
 *      Author: Kevin
 */

#ifndef SIMPLE_MATH_H_
#define SIMPLE_MATH_H_

#define lesser(a, b) (((a)<(b))?(a):(b))
#define greater(a, b) (((a)>(b))?(a):(b))
#define distance(a, b) (((a)>(b))?((a)-(b)):((b)-(a)))
#define clip(a, b, t) (((t)<(a))?(a):((((t)>(b))?(b):(t))))

#endif /* SIMPLE_MATH_H_ */
