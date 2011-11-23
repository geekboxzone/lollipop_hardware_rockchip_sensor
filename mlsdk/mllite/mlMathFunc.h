/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
#ifndef INVENSENSE_ML_MATH_FUNC_H__
#define INVENSENSE_ML_MATH_FUNC_H__


#define NUM_ROTATION_MATRIX_ELEMENTS (9)
#define ROT_MATRIX_SCALE_LONG  (1073741824)
#define ROT_MATRIX_SCALE_FLOAT (1073741824.0f)
#define ROT_MATRIX_LONG_TO_FLOAT( longval ) \
    ((float) ((longval) / ROT_MATRIX_SCALE_FLOAT ))
#define SIGNM(k)((int)(k)&1?-1:1)

#ifdef __cplusplus
extern "C" {
#endif
    struct filter_long {
        int length;
        const long * b;
        const long * a;
        long  * x;
        long  * y;
    };

    void FilterLong(struct filter_long *state, long x);
    long q29_mult( long a, long b );
    long q30_mult( long a, long b );
    void MLQMult(const long *q1, const long *q2, long *qProd);
    void MLQAdd(long *q1, long *q2, long *qSum);
    void MLQNormalize(long *q);
    void MLQInvert(const long *q, long *qInverted);
    void MLQMultf(const float *q1, const float *q2, float *qProd);
    void MLQAddf(float *q1, float *q2, float *qSum);
    void MLQNormalizef(float *q);
    void MLNorm4(float *q);
    void MLQInvertf(const float *q, float *qInverted);
    void quaternionToRotationMatrix( const long *quat, long *rot );
    unsigned char *Long32ToBig8(long x, unsigned char *big8);
    unsigned char *Short16ToBig8(short x, unsigned char *big8);
    float matDet(float *p,int *n);
    void matDetInc(float *a,float *b,int *n,int x,int y);
    double matDetd(double *p,int *n);
    void matDetIncd(double *a,double *b,int *n,int x,int y);
    float MLWrap(float ang);
    float MLAngDiff(float ang1, float ang2);

#ifdef __cplusplus
}
#endif

#endif // INVENSENSE_ML_MATH_FUNC_H__
