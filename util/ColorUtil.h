#ifndef COLORUTIL_H_
#define COLORUTIL_H_
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QVector>

/** Holds the value of a single color image pixel in 24-bit RGB format. */
typedef struct
{
    uchar r=0;
    uchar g=0;
    uchar b=0;
} Pixel;

struct ColorUtil
{
	ColorUtil();
    ~ColorUtil(){}

    QVector<QColor> heightMapPalette;
    QVector<QColor> heatMapPalette;

    QPen pen;
    QPen penDashed;
    QPen penThick;
    QPen penThin;
    QPen penThinDashed;
    QPen penWhite;
    QPen penWhiteDashed;
    QPen penWhiteThin;
    QPen penWhiteThick;

    QPen penGray;
    QPen penGrayDashed;
    QPen penGrayThin;
    QPen penGrayThinDashed;
    QPen penGrayThick;

    QPen penLightGray;
    QPen penLightGrayDashed;
    QPen penLightGrayThin;
    QPen penDarkGray;
    QPen penDarkGrayDashed;
    QPen penDarkGrayThin;
    QPen penDarkGrayThinDashed;
    QPen penRed;
    QPen penRedDashed;
    QPen penRedThin;
    QPen penRedThinDashed;
    QPen penRedThick;
    QPen penYellow;
    QPen penYellowThick;
    QPen penOrange;
    QPen penOrangeThick;
    QPen penBlue;
    QPen penBlueThin;
    QPen penBlueThick;
    QPen penGreen;
    QPen penGreenThick;
    QPen penGreenThin;


    QBrush brush;
    QBrush brushGray;
    QBrush brushLightGray;
    QBrush brushDarkGray;
    QBrush brushWhite;
    QBrush brushYellow;
    QBrush brushOrange;
    QBrush brushRed;
    QBrush brushBlue;
    QBrush brushMagenta;
    QBrush brushGreen;

    QColor getHeightMapColor(double v, double min, double max);
    QColor getHeatMapColor(double v, double min, double max);

	QColor sampleRedColor();
	QColor sampleBlueColor();
	QColor sampleUniformColor();
	QColor sampleNonRBColor();
};

extern ColorUtil colorUtil;

#endif /* COLORUTIL_H_ */

