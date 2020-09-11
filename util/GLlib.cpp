#include "GLlib.h"
#include <math.h>
#include <GL/glu.h>

namespace GLlib
{
    // 1: top left front
    // 2: bottom left front
    // 3: top right front
    // 4: bottom right front
    // 5: top right back
    // 6: bottom right back
    // 7: top left back
    // 8: bottom left back
    void drawBox(
                    double x1, double y1, double z1,
                    double x2, double y2, double z2,
                    double x3, double y3, double z3,
                    double x4, double y4, double z4,
                    double x5, double y5, double z5,
                    double x6, double y6, double z6,
                    double x7, double y7, double z7,
                    double x8, double y8, double z8
                    )
    {
        glBegin(GL_QUAD_STRIP);
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glVertex3f(x3, y3, z3);
        glVertex3f(x4, y4, z4);
        glVertex3f(x5, y5, z5);
        glVertex3f(x6, y6, z6);
        glVertex3f(x7, y7, z7);
        glVertex3f(x8, y8, z8);
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glEnd();

        glBegin(GL_QUADS);
        glVertex3f(x1, y1, z1);
        glVertex3f(x3, y3, z3);
        glVertex3f(x5, y5, z5);
        glVertex3f(x7, y7, z7);
        glVertex3f(x2, y2, z2);
        glVertex3f(x4, y4, z4);
        glVertex3f(x6, y6, z6);
        glVertex3f(x8, y8, z8);
        glEnd();

        glColor3f(0.3, 0.3, 0.3);
        glLineWidth(2);
        glBegin( GL_LINE_STRIP );
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glVertex3f(x4, y4, z4);
        glVertex3f(x3, y3, z3);
        glVertex3f(x1, y1, z1);
        glVertex3f(x7, y7, z7);
        glVertex3f(x8, y8, z8);
        glVertex3f(x6, y6, z6);
        glVertex3f(x5, y5, z5);
        glVertex3f(x7, y7, z7);
        glEnd();

        glBegin(GL_LINES);
        glVertex3f(x3, y3, z3);
        glVertex3f(x5, y5, z5);
        glVertex3f(x4, y4, z4);
        glVertex3f(x6, y6, z6);
        glVertex3f(x2, y2, z2);
        glVertex3f(x8, y8, z8);
        glEnd();
    }

    // Draws a cuboid with half extents x, y and z.
    void drawBox(float x, float y, float z)
    {
        glBegin( GL_QUAD_STRIP );
        glVertex3f(x, y, z);
        glVertex3f(x, y, -z);
        glVertex3f(x, -y, z);
        glVertex3f(x, -y, -z);
        glVertex3f(-x, -y, z);
        glVertex3f(-x, -y, -z);
        glVertex3f(-x, y, z);
        glVertex3f(-x, y, -z);
        glVertex3f(x, y, z);
        glVertex3f(x, y, -z);
        glEnd();

        glBegin( GL_QUADS );
        glVertex3f(x, y, z);
        glVertex3f(x, -y, z);
        glVertex3f(-x, -y, z);
        glVertex3f(-x, y, z);
        glVertex3f(x, y, -z);
        glVertex3f(x, -y, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(-x, y, -z);
        glEnd();

        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin( GL_LINE_STRIP );
        glVertex3f(x, y, z);
        glVertex3f(x, y, -z);
        glVertex3f(x, -y, -z);
        glVertex3f(x, -y, z);
        glVertex3f(x, y, z);
        glVertex3f(-x, y, z);
        glVertex3f(-x, y, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(-x, -y, z);
        glVertex3f(-x, y, z);
        glEnd();

        glBegin( GL_LINES );
        glVertex3f(x, -y, z);
        glVertex3f(-x, -y, z);
        glVertex3f(x, -y, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(x, y, -z);
        glVertex3f(-x, y, -z);
        glEnd();
    }

    // Draws a cuboid with half extents x, y and z.
    void drawWireFrame(float x, float y, float z)
    {
        glColor3f(0.3, 0.3, 0.3);
        glLineWidth(1);
        glBegin( GL_LINE_STRIP );
        glVertex3f(x, y, z);
        glVertex3f(x, y, -z);
        glVertex3f(x, -y, -z);
        glVertex3f(x, -y, z);
        glVertex3f(x, y, z);
        glVertex3f(-x, y, z);
        glVertex3f(-x, y, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(-x, -y, z);
        glVertex3f(-x, y, z);
        glEnd();

        glBegin( GL_LINES );
        glVertex3f(x, -y, z);
        glVertex3f(-x, -y, z);
        glVertex3f(x, -y, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(x, y, -z);
        glVertex3f(-x, y, -z);
        glEnd();
    }

    // Draws a quad with x,y top left corner and width and height.
    void drawQuad(float x, float y, float w, float h)
    {
        glBegin( GL_QUADS );
        glVertex3f(x, y, 0.001);
        glVertex3f(x+w, y, 0.001);
        glVertex3f(x+w, y-h, 0.001);
        glVertex3f(x, y-h, 0.001);
        glEnd();

        glColor3f(0.3, 0.3, 0.3);
        glLineWidth(2);
        glBegin( GL_LINE_STRIP );
        glVertex3f(x, y, 0.001);
        glVertex3f(x+w, y, 0.001);
        glVertex3f(x+w, y-h, 0.001);
        glVertex3f(x, y-h, 0.001);
        glEnd();
    }

    // Draws a sphere.
    void drawSphere(float radius)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluSphere(quadric, radius, 32, 32);
    }

    // Draws a sphere with a camera plane aligned border around it (you wish).
    void drawBorderedSphere(float radius)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluSphere(quadric, radius, 32, 32);
    }

    // Draws a circle in the xy plane.
    // http://slabode.exofire.net/circle_draw.shtml
    void drawCircle(float radius)
    {
        int slices = 32;

        float theta = 2 * 3.1415926 / float(slices);
        float c = cos(theta);
        float s = sin(theta);
        float t;

        float x = radius; //we start at angle = 0
        float y = 0;

        glLineWidth(2);
        glColor3f(0, 0 , 0);
        glBegin(GL_LINE_LOOP);
        for(int ii = 0; ii < slices; ii++)
        {
            glVertex3f(x, y, 0);

            //apply the rotation matrix
            t = x;
            x = c * x - s * y;
            y = s * t + c * y;
        }
        glEnd();
    }

    // Draws a filled circle in the xy plane.
    // http://slabode.exofire.net/circle_draw.shtml
    void drawFilledCircle(float radius)
    {
        int slices = 32;

        float theta = 2 * 3.1415926 / float(slices);
        float c = cos(theta);
        float s = sin(theta);
        float t;

        float x = radius; // start at angle = 0
        float y = 0;

        glBegin(GL_TRIANGLE_FAN);
        //glColor3f(r,g,b);
        glVertex3f(0, 0, 0);
        for(int ii = 0; ii <= slices; ii++)
        {
            glVertex3f(x, y, 0);
            t = x;
            x = c*x-s*y; // rotation matrix
            y = s*t+c*y; // rotation matrix
        }
        glEnd();
    }


    // Draws a filled circle in the xy plane.
    // http://slabode.exofire.net/circle_draw.shtml
    void drawFilledCircleStack(float radius)
    {
        drawFilledCircle(radius);

        glPushMatrix();
        glRotated(90, 1, 0, 0);
        drawFilledCircle(radius);
        glPopMatrix();

        glPushMatrix();
        glRotated(90, 0, 1, 0);
        drawFilledCircle(radius);
        glPopMatrix();
    }

    // Draws a coordinate frame.
    void drawFrame(float size, int lw)
    {
        glLineWidth(lw);
        glBegin( GL_LINES );
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(size, 0.0f, 0.0f);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, size, 0.0f);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, size);
        glEnd();
    }

    // Draws a coordinate frame.
    void drawFrame(float sx, float sy, float sz)
    {
        glLineWidth(2);
        glBegin( GL_LINES );
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(sx, 0.0f, 0.0f);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, sy, 0.0f);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, sz);
        glEnd();
    }


    // Draws an arrow that points along the x axis.
    void drawArrow(float length, float radius)
    {
        static GLUquadric* quadric = gluNewQuadric();

        if (radius < 0.0)
                radius = 0.05 * length;

        double headLength = 0.4*length < 0.15 ? 0.4*length : 0.15;
        double arrowLength = length - headLength;

        glPushMatrix();
        glRotated(90, 0, 1.0, 0);
        gluCylinder(quadric, radius, radius, arrowLength, 32, 1);
        glTranslatef(0.0, 0.0, arrowLength);
        gluCylinder(quadric, 1.5*radius, 0.0, headLength, 32, 1);
        glPopMatrix();
    }

    // Draws a cross.
    void drawCross(float size)
    {
        glTranslatef(0, 0, 0.001);
        glBegin( GL_QUADS );
        glVertex3f(size, 0.35*size, 0);
        glVertex3f(-size, 0.35*size, 0);
        glVertex3f(-size, -0.35*size, 0);
        glVertex3f(size, -0.35*size, 0);
        glEnd();
        glTranslatef(0, 0, 0.001);
        glBegin( GL_QUADS );
        glVertex3f(0.35*size, size, 0);
        glVertex3f(-0.35*size, size, 0);
        glVertex3f(-0.35*size, -size, 0);
        glVertex3f(0.35*size, -size, 0);
        glEnd();
        glTranslatef(0, 0, -0.002);
    }

    void drawCone(float bottomRadius, float topRadius, float height)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluCylinder(quadric, bottomRadius, topRadius, height, 32, 32);
        drawFilledCircle(bottomRadius);
        glTranslated(0, 0, height);
        drawFilledCircle(topRadius);
        drawCircle(topRadius);
        glTranslated(0, 0, -height);
        drawCircle(bottomRadius);
    }

    void drawCyclinder(float radius, float height)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluCylinder(quadric, radius, radius, height, 32, 32);
        drawFilledCircle(radius);
        glTranslated(0, 0, height);
        drawFilledCircle(radius);
        drawCircle(radius);
        glTranslated(0, 0, -height);
        drawCircle(radius);
    }
}
