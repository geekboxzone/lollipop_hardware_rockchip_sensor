/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlglyph.h 4552 2011-01-21 05:56:13Z mcaramello $
 *
 *******************************************************************************/

#ifndef MLGLYPH_H
#define MLGLYPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

    /* ------------ */
    /* - Defines. - */
    /* ------------ */

    /**
     *  @struct tMLGlyphData
     *  @brief  Describes the data to be used by the character recognition 
     *          algorithm.  When training and recognizing characters, data 
     *          is stored and read from this data container.
     *
     *  @param  yGlyph
     *  @param  xGlyph
     *  @param  GlyphLen
     *  @param  features
     *  @param  gestures
     *  @param  segments
     *  @param  library
     *  @param  libraryLength
     *  @param  probs
     *  @param  finalGesture
     *  @param  updatingGlyph
     *  @param  speedThresh
     *  @param  probFinal
     *  @param  minProb
    **/
    typedef struct {

        double yGlyph[512];
        double xGlyph[512];
        unsigned short GlyphLen;
        unsigned short features[32];
        unsigned short gestures[256];
        unsigned short segments[256];
        unsigned short library[256][32];
        unsigned short libraryLength;
        double probs[256][4];
        unsigned short finalGesture;
        unsigned char updatingGlyph;
        unsigned short speedThresh;
        double probFinal;
        double minProb;

    } tMLGlyphData,     // new type definition
      MLGLYPH_Data_t;   // background-compatible type definition

    /* --------------------- */
    /* - Function p-types. - */
    /* --------------------- */


    tMLError MLAddGlyph(unsigned short GlyphID);
    tMLError MLBestGlyph(unsigned short *finalGesture);
    tMLError MLSetGlyphSpeedThresh(unsigned short speed);
    tMLError MLStartGlyph(void);
    tMLError MLStopGlyph(void);
    tMLError MLGetGlyph(int index, int *x, int *y);
    tMLError MLGetGlyphLength(unsigned short *length);
    tMLError MLClearGlyph(void);
    tMLError MLLoadGlyphs(unsigned char *libraryData);
    tMLError MLStoreGlyphs(unsigned char *libraryData, unsigned short *length);
    tMLError MLEnableGlyph(void);
    tMLError MLSetGlyphProbThresh(unsigned short prob);
    tMLError MLGetLibraryLength(unsigned short *length);
    tMLError MLResetGlyphLibrary();

#ifdef __cplusplus
}
#endif

#endif /* MLGLYPH_H */
