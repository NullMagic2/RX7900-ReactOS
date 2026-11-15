/*
Module: rx7900.opengl.icd.dispatch
Purpose: The 336-entry OpenGL 1.1 dispatch table the ReactOS opengl32 loader
installs through DrvSetContext, forwarding to the current Context.
Created: 2026-10-23
Architecture: Generated from the pinned ReactOS a97c8a2
dll/opengl/opengl32/icd.h GLDISPATCHTABLE field order (icd_load contract). Each
thunk forwards to the thread-current Context (icd_current, in icd.cpp); calls
outside the fixed-function subset are safe no-ops that leave GL state unchanged.
Regenerate with scratchpad gen_dispatch.py against icd.h; the raster-position,
pixel-rectangle, pixel-map and polygon-stipple thunks (2026-10-24) keep its form.
GL types and the GLCLTPROCTABLE layout come from the ReactOS opengl32 headers.
*/
#include <windows.h>
#include <GL/gl.h>

#include "icd_table.hpp"
#include "../gl_context.hpp"

namespace rx7900::opengl { Context* icd_current() noexcept; }
using rx7900::opengl::icd_current;

extern "C" {


static void GLAPIENTRY icd_NewList(GLuint a0, GLenum a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->new_list(a0,a1); }
}

static void GLAPIENTRY icd_EndList(void) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->end_list(); }
}

static void GLAPIENTRY icd_CallList(GLuint a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->call_list(a0); }
}

static void GLAPIENTRY icd_CallLists(GLsizei a0, GLenum a1, const GLvoid * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->call_lists(a0,a1,a2); }
}

static void GLAPIENTRY icd_DeleteLists(GLuint a0, GLsizei a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->delete_lists(a0,a1); }
}

static GLuint GLAPIENTRY icd_GenLists(GLsizei a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { return C->gen_lists(a0); }
    return 0;
}

static void GLAPIENTRY icd_ListBase(GLuint a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->list_base(a0); }
}

static void GLAPIENTRY icd_Begin(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->begin(a0); }
}

static void GLAPIENTRY icd_Bitmap(GLsizei a0, GLsizei a1, GLfloat a2, GLfloat a3, GLfloat a4, GLfloat a5, const GLubyte * a6) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->bitmap(a0,a1,a2,a3,a4,a5,a6); }
}

static void GLAPIENTRY icd_Color3b(GLbyte a0, GLbyte a1, GLbyte a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/127),((double)a1/127),((double)a2/127),1.0); }
}

static void GLAPIENTRY icd_Color3bv(const GLbyte * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/127),((double)a0[1]/127),((double)a0[2]/127),1.0); }
}

static void GLAPIENTRY icd_Color3d(GLdouble a0, GLdouble a1, GLdouble a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_Color3dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_Color3f(GLfloat a0, GLfloat a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_Color3fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_Color3i(GLint a0, GLint a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/2.14748e+09),((double)a1/2.14748e+09),((double)a2/2.14748e+09),1.0); }
}

static void GLAPIENTRY icd_Color3iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/2.14748e+09),((double)a0[1]/2.14748e+09),((double)a0[2]/2.14748e+09),1.0); }
}

static void GLAPIENTRY icd_Color3s(GLshort a0, GLshort a1, GLshort a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/32767),((double)a1/32767),((double)a2/32767),1.0); }
}

static void GLAPIENTRY icd_Color3sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/32767),((double)a0[1]/32767),((double)a0[2]/32767),1.0); }
}

static void GLAPIENTRY icd_Color3ub(GLubyte a0, GLubyte a1, GLubyte a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/255),((double)a1/255),((double)a2/255),1.0); }
}

static void GLAPIENTRY icd_Color3ubv(const GLubyte * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/255),((double)a0[1]/255),((double)a0[2]/255),1.0); }
}

static void GLAPIENTRY icd_Color3ui(GLuint a0, GLuint a1, GLuint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/4.29497e+09),((double)a1/4.29497e+09),((double)a2/4.29497e+09),1.0); }
}

static void GLAPIENTRY icd_Color3uiv(const GLuint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/4.29497e+09),((double)a0[1]/4.29497e+09),((double)a0[2]/4.29497e+09),1.0); }
}

static void GLAPIENTRY icd_Color3us(GLushort a0, GLushort a1, GLushort a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/65535),((double)a1/65535),((double)a2/65535),1.0); }
}

static void GLAPIENTRY icd_Color3usv(const GLushort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/65535),((double)a0[1]/65535),((double)a0[2]/65535),1.0); }
}

static void GLAPIENTRY icd_Color4b(GLbyte a0, GLbyte a1, GLbyte a2, GLbyte a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/127),((double)a1/127),((double)a2/127),((double)a3/127)); }
}

static void GLAPIENTRY icd_Color4bv(const GLbyte * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/127),((double)a0[1]/127),((double)a0[2]/127),((double)a0[3]/127)); }
}

static void GLAPIENTRY icd_Color4d(GLdouble a0, GLdouble a1, GLdouble a2, GLdouble a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_Color4dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_Color4f(GLfloat a0, GLfloat a1, GLfloat a2, GLfloat a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_Color4fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_Color4i(GLint a0, GLint a1, GLint a2, GLint a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/2.14748e+09),((double)a1/2.14748e+09),((double)a2/2.14748e+09),((double)a3/2.14748e+09)); }
}

static void GLAPIENTRY icd_Color4iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/2.14748e+09),((double)a0[1]/2.14748e+09),((double)a0[2]/2.14748e+09),((double)a0[3]/2.14748e+09)); }
}

static void GLAPIENTRY icd_Color4s(GLshort a0, GLshort a1, GLshort a2, GLshort a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/32767),((double)a1/32767),((double)a2/32767),((double)a3/32767)); }
}

static void GLAPIENTRY icd_Color4sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/32767),((double)a0[1]/32767),((double)a0[2]/32767),((double)a0[3]/32767)); }
}

static void GLAPIENTRY icd_Color4ub(GLubyte a0, GLubyte a1, GLubyte a2, GLubyte a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/255),((double)a1/255),((double)a2/255),((double)a3/255)); }
}

static void GLAPIENTRY icd_Color4ubv(const GLubyte * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/255),((double)a0[1]/255),((double)a0[2]/255),((double)a0[3]/255)); }
}

static void GLAPIENTRY icd_Color4ui(GLuint a0, GLuint a1, GLuint a2, GLuint a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/4.29497e+09),((double)a1/4.29497e+09),((double)a2/4.29497e+09),((double)a3/4.29497e+09)); }
}

static void GLAPIENTRY icd_Color4uiv(const GLuint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/4.29497e+09),((double)a0[1]/4.29497e+09),((double)a0[2]/4.29497e+09),((double)a0[3]/4.29497e+09)); }
}

static void GLAPIENTRY icd_Color4us(GLushort a0, GLushort a1, GLushort a2, GLushort a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0/65535),((double)a1/65535),((double)a2/65535),((double)a3/65535)); }
}

static void GLAPIENTRY icd_Color4usv(const GLushort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color(((double)a0[0]/65535),((double)a0[1]/65535),((double)a0[2]/65535),((double)a0[3]/65535)); }
}

static void GLAPIENTRY icd_EdgeFlag(GLboolean a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->edge_flag(a0 != 0); }
}

static void GLAPIENTRY icd_EdgeFlagv(const GLboolean * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->edge_flag(a0[0] != 0); }
}

static void GLAPIENTRY icd_End(void) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->end(); }
}

static void GLAPIENTRY icd_Indexd(GLdouble a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Indexdv(const GLdouble * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Indexf(GLfloat a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Indexfv(const GLfloat * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Indexi(GLint a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Indexiv(const GLint * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Indexs(GLshort a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Indexsv(const GLshort * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Normal3b(GLbyte a0, GLbyte a1, GLbyte a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0,(double)a1,(double)a2); }
}

static void GLAPIENTRY icd_Normal3bv(const GLbyte * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0[0],(double)a0[1],(double)a0[2]); }
}

static void GLAPIENTRY icd_Normal3d(GLdouble a0, GLdouble a1, GLdouble a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0,(double)a1,(double)a2); }
}

static void GLAPIENTRY icd_Normal3dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0[0],(double)a0[1],(double)a0[2]); }
}

static void GLAPIENTRY icd_Normal3f(GLfloat a0, GLfloat a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0,(double)a1,(double)a2); }
}

static void GLAPIENTRY icd_Normal3fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0[0],(double)a0[1],(double)a0[2]); }
}

static void GLAPIENTRY icd_Normal3i(GLint a0, GLint a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0,(double)a1,(double)a2); }
}

static void GLAPIENTRY icd_Normal3iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0[0],(double)a0[1],(double)a0[2]); }
}

static void GLAPIENTRY icd_Normal3s(GLshort a0, GLshort a1, GLshort a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0,(double)a1,(double)a2); }
}

static void GLAPIENTRY icd_Normal3sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal((double)a0[0],(double)a0[1],(double)a0[2]); }
}

static void GLAPIENTRY icd_RasterPos2d(GLdouble a0, GLdouble a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_RasterPos2dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_RasterPos2f(GLfloat a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_RasterPos2fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_RasterPos2i(GLint a0, GLint a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_RasterPos2iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_RasterPos2s(GLshort a0, GLshort a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_RasterPos2sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_RasterPos3d(GLdouble a0, GLdouble a1, GLdouble a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_RasterPos3dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_RasterPos3f(GLfloat a0, GLfloat a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_RasterPos3fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_RasterPos3i(GLint a0, GLint a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_RasterPos3iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_RasterPos3s(GLshort a0, GLshort a1, GLshort a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_RasterPos3sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_RasterPos4d(GLdouble a0, GLdouble a1, GLdouble a2, GLdouble a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_RasterPos4dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_RasterPos4f(GLfloat a0, GLfloat a1, GLfloat a2, GLfloat a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_RasterPos4fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_RasterPos4i(GLint a0, GLint a1, GLint a2, GLint a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_RasterPos4iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_RasterPos4s(GLshort a0, GLshort a1, GLshort a2, GLshort a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_RasterPos4sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->raster_pos((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_Rectd(GLdouble a0, GLdouble a1, GLdouble a2, GLdouble a3) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
}

static void GLAPIENTRY icd_Rectdv(const GLdouble * a0, const GLdouble * a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_Rectf(GLfloat a0, GLfloat a1, GLfloat a2, GLfloat a3) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
}

static void GLAPIENTRY icd_Rectfv(const GLfloat * a0, const GLfloat * a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_Recti(GLint a0, GLint a1, GLint a2, GLint a3) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
}

static void GLAPIENTRY icd_Rectiv(const GLint * a0, const GLint * a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_Rects(GLshort a0, GLshort a1, GLshort a2, GLshort a3) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
}

static void GLAPIENTRY icd_Rectsv(const GLshort * a0, const GLshort * a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_TexCoord1d(GLdouble a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,0.0,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord1dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],0.0,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord1f(GLfloat a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,0.0,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord1fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],0.0,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord1i(GLint a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,0.0,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord1iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],0.0,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord1s(GLshort a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,0.0,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord1sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],0.0,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord2d(GLdouble a0, GLdouble a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord2dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord2f(GLfloat a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord2fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord2i(GLint a0, GLint a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord2iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord2s(GLshort a0, GLshort a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord2sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_TexCoord3d(GLdouble a0, GLdouble a1, GLdouble a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_TexCoord3dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_TexCoord3f(GLfloat a0, GLfloat a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_TexCoord3fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_TexCoord3i(GLint a0, GLint a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_TexCoord3iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_TexCoord3s(GLshort a0, GLshort a1, GLshort a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_TexCoord3sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_TexCoord4d(GLdouble a0, GLdouble a1, GLdouble a2, GLdouble a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_TexCoord4dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_TexCoord4f(GLfloat a0, GLfloat a1, GLfloat a2, GLfloat a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_TexCoord4fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_TexCoord4i(GLint a0, GLint a1, GLint a2, GLint a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_TexCoord4iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_TexCoord4s(GLshort a0, GLshort a1, GLshort a2, GLshort a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_TexCoord4sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_Vertex2d(GLdouble a0, GLdouble a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_Vertex2dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_Vertex2f(GLfloat a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_Vertex2fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_Vertex2i(GLint a0, GLint a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_Vertex2iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_Vertex2s(GLshort a0, GLshort a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,0.0,1.0); }
}

static void GLAPIENTRY icd_Vertex2sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],0.0,1.0); }
}

static void GLAPIENTRY icd_Vertex3d(GLdouble a0, GLdouble a1, GLdouble a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_Vertex3dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_Vertex3f(GLfloat a0, GLfloat a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_Vertex3fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_Vertex3i(GLint a0, GLint a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_Vertex3iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_Vertex3s(GLshort a0, GLshort a1, GLshort a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,(double)a2,1.0); }
}

static void GLAPIENTRY icd_Vertex3sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],(double)a0[2],1.0); }
}

static void GLAPIENTRY icd_Vertex4d(GLdouble a0, GLdouble a1, GLdouble a2, GLdouble a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_Vertex4dv(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_Vertex4f(GLfloat a0, GLfloat a1, GLfloat a2, GLfloat a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_Vertex4fv(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_Vertex4i(GLint a0, GLint a1, GLint a2, GLint a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_Vertex4iv(const GLint * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_Vertex4s(GLshort a0, GLshort a1, GLshort a2, GLshort a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0,(double)a1,(double)a2,(double)a3); }
}

static void GLAPIENTRY icd_Vertex4sv(const GLshort * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex((double)a0[0],(double)a0[1],(double)a0[2],(double)a0[3]); }
}

static void GLAPIENTRY icd_ClipPlane(GLenum a0, const GLdouble * a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_ColorMaterial(GLenum a0, GLenum a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color_material(a0,a1); }
}

static void GLAPIENTRY icd_CullFace(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->cull_face(a0); }
}

static void GLAPIENTRY icd_Fogf(GLenum a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a1}; C->fog(a0,f); }
}

static void GLAPIENTRY icd_Fogfv(GLenum a0, const GLfloat * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a1[i]; C->fog(a0,f); }
}

static void GLAPIENTRY icd_Fogi(GLenum a0, GLint a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a1}; C->fog(a0,f); }
}

static void GLAPIENTRY icd_Fogiv(GLenum a0, const GLint * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a1[i]; C->fog(a0,f); }
}

static void GLAPIENTRY icd_FrontFace(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->front_face(a0); }
}

static void GLAPIENTRY icd_Hint(GLenum a0, GLenum a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->hint(a0,a1); }
}

static void GLAPIENTRY icd_Lightf(GLenum a0, GLenum a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a2}; C->light(a0,a1,f); }
}

static void GLAPIENTRY icd_Lightfv(GLenum a0, GLenum a1, const GLfloat * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a2[i]; C->light(a0,a1,f); }
}

static void GLAPIENTRY icd_Lighti(GLenum a0, GLenum a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a2}; C->light(a0,a1,f); }
}

static void GLAPIENTRY icd_Lightiv(GLenum a0, GLenum a1, const GLint * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a2[i]; C->light(a0,a1,f); }
}

static void GLAPIENTRY icd_LightModelf(GLenum a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a1}; C->light_model(a0,f); }
}

static void GLAPIENTRY icd_LightModelfv(GLenum a0, const GLfloat * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a1[i]; C->light_model(a0,f); }
}

static void GLAPIENTRY icd_LightModeli(GLenum a0, GLint a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a1}; C->light_model(a0,f); }
}

static void GLAPIENTRY icd_LightModeliv(GLenum a0, const GLint * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a1[i]; C->light_model(a0,f); }
}

static void GLAPIENTRY icd_LineStipple(GLint a0, GLushort a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->line_stipple(a0,a1); }
}

static void GLAPIENTRY icd_LineWidth(GLfloat a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->line_width(a0); }
}

static void GLAPIENTRY icd_Materialf(GLenum a0, GLenum a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a2}; C->material(a0,a1,f); }
}

static void GLAPIENTRY icd_Materialfv(GLenum a0, GLenum a1, const GLfloat * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a2[i]; C->material(a0,a1,f); }
}

static void GLAPIENTRY icd_Materiali(GLenum a0, GLenum a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a2}; C->material(a0,a1,f); }
}

static void GLAPIENTRY icd_Materialiv(GLenum a0, GLenum a1, const GLint * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a2[i]; C->material(a0,a1,f); }
}

static void GLAPIENTRY icd_PointSize(GLfloat a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->point_size(a0); }
}

static void GLAPIENTRY icd_PolygonMode(GLenum a0, GLenum a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->polygon_mode(a0,a1); }
}

static void GLAPIENTRY icd_PolygonStipple(const GLubyte * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->polygon_stipple(a0); }
}

static void GLAPIENTRY icd_Scissor(GLint a0, GLint a1, GLsizei a2, GLsizei a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->scissor(a0,a1,a2,a3); }
}

static void GLAPIENTRY icd_ShadeModel(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->shade_model(a0); }
}

static void GLAPIENTRY icd_TexParameterf(GLenum a0, GLenum a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a2}; C->tex_parameter(a0,a1,f); }
}

static void GLAPIENTRY icd_TexParameterfv(GLenum a0, GLenum a1, const GLfloat * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]={(float)a2[0],0,0,0}; C->tex_parameter(a0,a1,f); }
}

static void GLAPIENTRY icd_TexParameteri(GLenum a0, GLenum a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a2}; C->tex_parameter(a0,a1,f); }
}

static void GLAPIENTRY icd_TexParameteriv(GLenum a0, GLenum a1, const GLint * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]={(float)a2[0],0,0,0}; C->tex_parameter(a0,a1,f); }
}

static void GLAPIENTRY icd_TexImage1D(GLenum a0, GLint a1, GLint a2, GLsizei a3, GLint a4, GLenum a5, GLenum a6, const GLvoid * a7) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
    (void)a6;
    (void)a7;
}

static void GLAPIENTRY icd_TexImage2D(GLenum a0, GLint a1, GLint a2, GLsizei a3, GLsizei a4, GLint a5, GLenum a6, GLenum a7, const GLvoid * a8) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_image_2d(a0,a1,a2,a3,a4,a5,a6,a7,a8); }
}

static void GLAPIENTRY icd_TexEnvf(GLenum a0, GLenum a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a2}; C->tex_env(a0,a1,f); }
}

static void GLAPIENTRY icd_TexEnvfv(GLenum a0, GLenum a1, const GLfloat * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a2[i]; C->tex_env(a0,a1,f); }
}

static void GLAPIENTRY icd_TexEnvi(GLenum a0, GLenum a1, GLint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[1]={(float)a2}; C->tex_env(a0,a1,f); }
}

static void GLAPIENTRY icd_TexEnviv(GLenum a0, GLenum a1, const GLint * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { float f[4]; for(int i=0;i<4;++i) f[i]=(float)a2[i]; C->tex_env(a0,a1,f); }
}

static void GLAPIENTRY icd_TexGend(GLenum a0, GLenum a1, GLdouble a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_TexGendv(GLenum a0, GLenum a1, const GLdouble * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_TexGenf(GLenum a0, GLenum a1, GLfloat a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_TexGenfv(GLenum a0, GLenum a1, const GLfloat * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_TexGeni(GLenum a0, GLenum a1, GLint a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_TexGeniv(GLenum a0, GLenum a1, const GLint * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_FeedbackBuffer(GLsizei a0, GLenum a1, GLfloat * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_SelectBuffer(GLsizei a0, GLuint * a1) noexcept {
    (void)a0;
    (void)a1;
}

static GLint GLAPIENTRY icd_RenderMode(GLenum a0) noexcept {
    (void)a0;
    return 0;
}

static void GLAPIENTRY icd_InitNames(void) noexcept {
}

static void GLAPIENTRY icd_LoadName(GLuint a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_PassThrough(GLfloat a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_PopName(void) noexcept {
}

static void GLAPIENTRY icd_PushName(GLuint a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_DrawBuffer(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->draw_buffer(a0); }
}

static void GLAPIENTRY icd_Clear(GLbitfield a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->clear(a0); }
}

static void GLAPIENTRY icd_ClearAccum(GLfloat a0, GLfloat a1, GLfloat a2, GLfloat a3) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
}

static void GLAPIENTRY icd_ClearIndex(GLfloat a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_ClearColor(GLclampf a0, GLclampf a1, GLclampf a2, GLclampf a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->clear_color(a0,a1,a2,a3); }
}

static void GLAPIENTRY icd_ClearStencil(GLint a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->clear_stencil(a0); }
}

static void GLAPIENTRY icd_ClearDepth(GLclampd a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->clear_depth(a0); }
}

static void GLAPIENTRY icd_StencilMask(GLuint a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->stencil_mask(a0); }
}

static void GLAPIENTRY icd_ColorMask(GLboolean a0, GLboolean a1, GLboolean a2, GLboolean a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color_mask(a0!=0,a1!=0,a2!=0,a3!=0); }
}

static void GLAPIENTRY icd_DepthMask(GLboolean a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->depth_mask(a0 != 0); }
}

static void GLAPIENTRY icd_IndexMask(GLuint a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Accum(GLenum a0, GLfloat a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_Disable(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->enable(a0,false); }
}

static void GLAPIENTRY icd_Enable(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->enable(a0,true); }
}

static void GLAPIENTRY icd_Finish(void) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->finish(); }
}

static void GLAPIENTRY icd_Flush(void) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->flush(); }
}

static void GLAPIENTRY icd_PopAttrib(void) noexcept {
}

static void GLAPIENTRY icd_PushAttrib(GLbitfield a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Map1d(GLenum a0, GLdouble a1, GLdouble a2, GLint a3, GLint a4, const GLdouble * a5) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
}

static void GLAPIENTRY icd_Map1f(GLenum a0, GLfloat a1, GLfloat a2, GLint a3, GLint a4, const GLfloat * a5) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
}

static void GLAPIENTRY icd_Map2d(GLenum a0, GLdouble a1, GLdouble a2, GLint a3, GLint a4, GLdouble a5, GLdouble a6, GLint a7, GLint a8, const GLdouble * a9) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
    (void)a6;
    (void)a7;
    (void)a8;
    (void)a9;
}

static void GLAPIENTRY icd_Map2f(GLenum a0, GLfloat a1, GLfloat a2, GLint a3, GLint a4, GLfloat a5, GLfloat a6, GLint a7, GLint a8, const GLfloat * a9) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
    (void)a6;
    (void)a7;
    (void)a8;
    (void)a9;
}

static void GLAPIENTRY icd_MapGrid1d(GLint a0, GLdouble a1, GLdouble a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_MapGrid1f(GLint a0, GLfloat a1, GLfloat a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_MapGrid2d(GLint a0, GLdouble a1, GLdouble a2, GLint a3, GLdouble a4, GLdouble a5) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
}

static void GLAPIENTRY icd_MapGrid2f(GLint a0, GLfloat a1, GLfloat a2, GLint a3, GLfloat a4, GLfloat a5) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
}

static void GLAPIENTRY icd_EvalCoord1d(GLdouble a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_EvalCoord1dv(const GLdouble * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_EvalCoord1f(GLfloat a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_EvalCoord1fv(const GLfloat * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_EvalCoord2d(GLdouble a0, GLdouble a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_EvalCoord2dv(const GLdouble * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_EvalCoord2f(GLfloat a0, GLfloat a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_EvalCoord2fv(const GLfloat * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_EvalMesh1(GLenum a0, GLint a1, GLint a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_EvalPoint1(GLint a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_EvalMesh2(GLenum a0, GLint a1, GLint a2, GLint a3, GLint a4) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
}

static void GLAPIENTRY icd_EvalPoint2(GLint a0, GLint a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_AlphaFunc(GLenum a0, GLclampf a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->alpha_func(a0,a1); }
}

static void GLAPIENTRY icd_BlendFunc(GLenum a0, GLenum a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->blend_func(a0,a1); }
}

static void GLAPIENTRY icd_LogicOp(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->logic_op(a0); }
}

static void GLAPIENTRY icd_StencilFunc(GLenum a0, GLint a1, GLuint a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->stencil_func(a0,a1,a2); }
}

static void GLAPIENTRY icd_StencilOp(GLenum a0, GLenum a1, GLenum a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->stencil_op(a0,a1,a2); }
}

static void GLAPIENTRY icd_DepthFunc(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->depth_func(a0); }
}

static void GLAPIENTRY icd_PixelZoom(GLfloat a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->pixel_zoom(a0,a1); }
}

static void GLAPIENTRY icd_PixelTransferf(GLenum a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->pixel_transfer(a0,a1); }
}

static void GLAPIENTRY icd_PixelTransferi(GLenum a0, GLint a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->pixel_transfer(a0,(GLfloat)a1); }
}

static void GLAPIENTRY icd_PixelStoref(GLenum a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->pixel_store(a0,(GLint)a1); }
}

static void GLAPIENTRY icd_PixelStorei(GLenum a0, GLint a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->pixel_store(a0,a1); }
}

static void GLAPIENTRY icd_PixelMapfv(GLenum a0, GLint a1, const GLfloat * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->pixel_map(a0,a1,a2); }
}

static void GLAPIENTRY icd_PixelMapuiv(GLenum a0, GLint a1, const GLuint * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (!C || a2 == nullptr || a1 < 1 || a1 > 256) { if (C) { C->pixel_map(a0,a1,nullptr); } return; }
    GLfloat values[256];
    const bool normalized = a0 > GL_PIXEL_MAP_S_TO_S;
    for (GLint i = 0; i < a1; ++i) { values[i] = normalized ? (GLfloat)((double)a2[i] / 4294967295.0) : (GLfloat)a2[i]; }
    C->pixel_map(a0,a1,values);
}

static void GLAPIENTRY icd_PixelMapusv(GLenum a0, GLint a1, const GLushort * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (!C || a2 == nullptr || a1 < 1 || a1 > 256) { if (C) { C->pixel_map(a0,a1,nullptr); } return; }
    GLfloat values[256];
    const bool normalized = a0 > GL_PIXEL_MAP_S_TO_S;
    for (GLint i = 0; i < a1; ++i) { values[i] = normalized ? (GLfloat)((double)a2[i] / 65535.0) : (GLfloat)a2[i]; }
    C->pixel_map(a0,a1,values);
}

static void GLAPIENTRY icd_ReadBuffer(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->read_buffer(a0); }
}

static void GLAPIENTRY icd_CopyPixels(GLint a0, GLint a1, GLsizei a2, GLsizei a3, GLenum a4) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->copy_pixels(a0,a1,a2,a3,a4); }
}

static void GLAPIENTRY icd_ReadPixels(GLint a0, GLint a1, GLsizei a2, GLsizei a3, GLenum a4, GLenum a5, GLvoid * a6) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->read_pixels(a0,a1,a2,a3,a4,a5,a6); }
}

static void GLAPIENTRY icd_DrawPixels(GLsizei a0, GLsizei a1, GLenum a2, GLenum a3, const GLvoid * a4) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->draw_pixels(a0,a1,a2,a3,a4); }
}

static void GLAPIENTRY icd_GetBooleanv(GLenum a0, GLboolean * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->get_booleanv(a0,a1); }
}

static void GLAPIENTRY icd_GetClipPlane(GLenum a0, GLdouble * a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_GetDoublev(GLenum a0, GLdouble * a1) noexcept {
    (void)a0;
    (void)a1;
}

static GLenum GLAPIENTRY icd_GetError(void) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { return C->get_error(); }
    return 0;
}

static void GLAPIENTRY icd_GetFloatv(GLenum a0, GLfloat * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->get_floatv(a0,a1); }
}

static void GLAPIENTRY icd_GetIntegerv(GLenum a0, GLint * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->get_integerv(a0,a1); }
}

static void GLAPIENTRY icd_GetLightfv(GLenum a0, GLenum a1, GLfloat * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetLightiv(GLenum a0, GLenum a1, GLint * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetMapdv(GLenum a0, GLenum a1, GLdouble * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetMapfv(GLenum a0, GLenum a1, GLfloat * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetMapiv(GLenum a0, GLenum a1, GLint * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetMaterialfv(GLenum a0, GLenum a1, GLfloat * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetMaterialiv(GLenum a0, GLenum a1, GLint * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetPixelMapfv(GLenum a0, GLfloat * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->get_pixel_map(a0,a1); }
}

static void GLAPIENTRY icd_GetPixelMapuiv(GLenum a0, GLuint * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (!C || a1 == nullptr) { return; }
    GLfloat values[256];
    const GLuint size = C->pixel_map_size(a0);
    if (size == 0 || size > 256) { return; }
    C->get_pixel_map(a0,values);
    const bool normalized = a0 > GL_PIXEL_MAP_S_TO_S;
    for (GLuint i = 0; i < size; ++i) { a1[i] = (GLuint)(normalized ? (double)values[i] * 4294967295.0 + 0.5 : (double)values[i]); }
}

static void GLAPIENTRY icd_GetPixelMapusv(GLenum a0, GLushort * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (!C || a1 == nullptr) { return; }
    GLfloat values[256];
    const GLuint size = C->pixel_map_size(a0);
    if (size == 0 || size > 256) { return; }
    C->get_pixel_map(a0,values);
    const bool normalized = a0 > GL_PIXEL_MAP_S_TO_S;
    for (GLuint i = 0; i < size; ++i) { a1[i] = (GLushort)(normalized ? (double)values[i] * 65535.0 + 0.5 : (double)values[i]); }
}

static void GLAPIENTRY icd_GetPolygonStipple(GLubyte * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->get_polygon_stipple(a0); }
}

static const GLubyte * GLAPIENTRY icd_GetString(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { return reinterpret_cast<const GLubyte*>(C->get_string(a0)); }
    return 0;
}

static void GLAPIENTRY icd_GetTexEnvfv(GLenum a0, GLenum a1, GLfloat * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetTexEnviv(GLenum a0, GLenum a1, GLint * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetTexGendv(GLenum a0, GLenum a1, GLdouble * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetTexGenfv(GLenum a0, GLenum a1, GLfloat * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetTexGeniv(GLenum a0, GLenum a1, GLint * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetTexImage(GLenum a0, GLint a1, GLenum a2, GLenum a3, GLvoid * a4) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
}

static void GLAPIENTRY icd_GetTexParameterfv(GLenum a0, GLenum a1, GLfloat * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetTexParameteriv(GLenum a0, GLenum a1, GLint * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_GetTexLevelParameterfv(GLenum a0, GLint a1, GLenum a2, GLfloat * a3) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
}

static void GLAPIENTRY icd_GetTexLevelParameteriv(GLenum a0, GLint a1, GLenum a2, GLint * a3) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
}

static GLboolean GLAPIENTRY icd_IsEnabled(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { return C->is_enabled(a0)?1:0; }
    return 0;
}

static GLboolean GLAPIENTRY icd_IsList(GLuint a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { return C->is_list(a0)?1:0; }
    return 0;
}

static void GLAPIENTRY icd_DepthRange(GLclampd a0, GLclampd a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->depth_range(a0,a1); }
}

static void GLAPIENTRY icd_Frustum(GLdouble a0, GLdouble a1, GLdouble a2, GLdouble a3, GLdouble a4, GLdouble a5) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->frustum(a0,a1,a2,a3,a4,a5); }
}

static void GLAPIENTRY icd_LoadIdentity(void) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->load_identity(); }
}

static void GLAPIENTRY icd_LoadMatrixf(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { double m[16]; for(int i=0;i<16;++i) m[i]=(double)a0[i]; C->load_matrix(m); }
}

static void GLAPIENTRY icd_LoadMatrixd(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { double m[16]; for(int i=0;i<16;++i) m[i]=(double)a0[i]; C->load_matrix(m); }
}

static void GLAPIENTRY icd_MatrixMode(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->matrix_mode(a0); }
}

static void GLAPIENTRY icd_MultMatrixf(const GLfloat * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { double m[16]; for(int i=0;i<16;++i) m[i]=(double)a0[i]; C->mult_matrix(m); }
}

static void GLAPIENTRY icd_MultMatrixd(const GLdouble * a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { double m[16]; for(int i=0;i<16;++i) m[i]=(double)a0[i]; C->mult_matrix(m); }
}

static void GLAPIENTRY icd_Ortho(GLdouble a0, GLdouble a1, GLdouble a2, GLdouble a3, GLdouble a4, GLdouble a5) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->ortho(a0,a1,a2,a3,a4,a5); }
}

static void GLAPIENTRY icd_PopMatrix(void) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->pop_matrix(); }
}

static void GLAPIENTRY icd_PushMatrix(void) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->push_matrix(); }
}

static void GLAPIENTRY icd_Rotated(GLdouble a0, GLdouble a1, GLdouble a2, GLdouble a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->rotate(a0,a1,a2,a3); }
}

static void GLAPIENTRY icd_Rotatef(GLfloat a0, GLfloat a1, GLfloat a2, GLfloat a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->rotate(a0,a1,a2,a3); }
}

static void GLAPIENTRY icd_Scaled(GLdouble a0, GLdouble a1, GLdouble a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->scale(a0,a1,a2); }
}

static void GLAPIENTRY icd_Scalef(GLfloat a0, GLfloat a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->scale(a0,a1,a2); }
}

static void GLAPIENTRY icd_Translated(GLdouble a0, GLdouble a1, GLdouble a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->translate(a0,a1,a2); }
}

static void GLAPIENTRY icd_Translatef(GLfloat a0, GLfloat a1, GLfloat a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->translate(a0,a1,a2); }
}

static void GLAPIENTRY icd_Viewport(GLint a0, GLint a1, GLsizei a2, GLsizei a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->viewport(a0,a1,a2,a3); }
}

static void GLAPIENTRY icd_ArrayElement(GLint a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->array_element(a0); }
}

static void GLAPIENTRY icd_BindTexture(GLenum a0, GLuint a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->bind_texture(a0,a1); }
}

static void GLAPIENTRY icd_ColorPointer(GLint a0, GLenum a1, GLsizei a2, const GLvoid * a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->color_pointer(a0,a1,a2,a3); }
}

static void GLAPIENTRY icd_DisableClientState(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->enable_client_state(a0,false); }
}

static void GLAPIENTRY icd_DrawArrays(GLenum a0, GLint a1, GLsizei a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->draw_arrays(a0,a1,a2); }
}

static void GLAPIENTRY icd_DrawElements(GLenum a0, GLsizei a1, GLenum a2, const GLvoid * a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->draw_elements(a0,a1,a2,a3); }
}

static void GLAPIENTRY icd_EdgeFlagPointer(GLsizei a0, const GLvoid * a1) noexcept {
    (void)a0;
    (void)a1;
}

static void GLAPIENTRY icd_EnableClientState(GLenum a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->enable_client_state(a0,true); }
}

static void GLAPIENTRY icd_IndexPointer(GLenum a0, GLsizei a1, const GLvoid * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_Indexub(GLubyte a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_Indexubv(const GLubyte * a0) noexcept {
    (void)a0;
}

static void GLAPIENTRY icd_InterleavedArrays(GLenum a0, GLsizei a1, const GLvoid * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_NormalPointer(GLenum a0, GLsizei a1, const GLvoid * a2) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->normal_pointer(a0,a1,a2); }
}

static void GLAPIENTRY icd_PolygonOffset(GLfloat a0, GLfloat a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->polygon_offset(a0,a1); }
}

static void GLAPIENTRY icd_TexCoordPointer(GLint a0, GLenum a1, GLsizei a2, const GLvoid * a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_coord_pointer(a0,a1,a2,a3); }
}

static void GLAPIENTRY icd_VertexPointer(GLint a0, GLenum a1, GLsizei a2, const GLvoid * a3) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->vertex_pointer(a0,a1,a2,a3); }
}

static GLboolean GLAPIENTRY icd_AreTexturesResident(GLsizei a0, const GLuint * a1, GLboolean * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    return 0;
}

static void GLAPIENTRY icd_CopyTexImage1D(GLenum a0, GLint a1, GLenum a2, GLint a3, GLint a4, GLsizei a5, GLint a6) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
    (void)a6;
}

static void GLAPIENTRY icd_CopyTexImage2D(GLenum a0, GLint a1, GLenum a2, GLint a3, GLint a4, GLsizei a5, GLsizei a6, GLint a7) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
    (void)a6;
    (void)a7;
}

static void GLAPIENTRY icd_CopyTexSubImage1D(GLenum a0, GLint a1, GLint a2, GLint a3, GLint a4, GLsizei a5) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
}

static void GLAPIENTRY icd_CopyTexSubImage2D(GLenum a0, GLint a1, GLint a2, GLint a3, GLint a4, GLint a5, GLsizei a6, GLsizei a7) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
    (void)a6;
    (void)a7;
}

static void GLAPIENTRY icd_DeleteTextures(GLsizei a0, const GLuint * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->delete_textures(a0,a1); }
}

static void GLAPIENTRY icd_GenTextures(GLsizei a0, GLuint * a1) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->gen_textures(a0,a1); }
}

static void GLAPIENTRY icd_GetPointerv(GLenum a0, GLvoid ** a1) noexcept {
    (void)a0;
    (void)a1;
}

static GLboolean GLAPIENTRY icd_IsTexture(GLuint a0) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { return C->is_texture(a0)?1:0; }
    return 0;
}

static void GLAPIENTRY icd_PrioritizeTextures(GLsizei a0, const GLuint * a1, const GLclampf * a2) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
}

static void GLAPIENTRY icd_TexSubImage1D(GLenum a0, GLint a1, GLint a2, GLsizei a3, GLenum a4, GLenum a5, const GLvoid * a6) noexcept {
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    (void)a4;
    (void)a5;
    (void)a6;
}

static void GLAPIENTRY icd_TexSubImage2D(GLenum a0, GLint a1, GLint a2, GLint a3, GLsizei a4, GLsizei a5, GLenum a6, GLenum a7, const GLvoid * a8) noexcept {
    rx7900::opengl::Context* C = icd_current();
    if (C) { C->tex_sub_image_2d(a0,a1,a2,a3,a4,a5,a6,a7,a8); }
}

static void GLAPIENTRY icd_PopClientAttrib(void) noexcept {
}

static void GLAPIENTRY icd_PushClientAttrib(GLbitfield a0) noexcept {
    (void)a0;
}

const GLCLTPROCTABLE rx7900_icd_proc_table = { OPENGL_VERSION_110_ENTRIES, {
    icd_NewList,
    icd_EndList,
    icd_CallList,
    icd_CallLists,
    icd_DeleteLists,
    icd_GenLists,
    icd_ListBase,
    icd_Begin,
    icd_Bitmap,
    icd_Color3b,
    icd_Color3bv,
    icd_Color3d,
    icd_Color3dv,
    icd_Color3f,
    icd_Color3fv,
    icd_Color3i,
    icd_Color3iv,
    icd_Color3s,
    icd_Color3sv,
    icd_Color3ub,
    icd_Color3ubv,
    icd_Color3ui,
    icd_Color3uiv,
    icd_Color3us,
    icd_Color3usv,
    icd_Color4b,
    icd_Color4bv,
    icd_Color4d,
    icd_Color4dv,
    icd_Color4f,
    icd_Color4fv,
    icd_Color4i,
    icd_Color4iv,
    icd_Color4s,
    icd_Color4sv,
    icd_Color4ub,
    icd_Color4ubv,
    icd_Color4ui,
    icd_Color4uiv,
    icd_Color4us,
    icd_Color4usv,
    icd_EdgeFlag,
    icd_EdgeFlagv,
    icd_End,
    icd_Indexd,
    icd_Indexdv,
    icd_Indexf,
    icd_Indexfv,
    icd_Indexi,
    icd_Indexiv,
    icd_Indexs,
    icd_Indexsv,
    icd_Normal3b,
    icd_Normal3bv,
    icd_Normal3d,
    icd_Normal3dv,
    icd_Normal3f,
    icd_Normal3fv,
    icd_Normal3i,
    icd_Normal3iv,
    icd_Normal3s,
    icd_Normal3sv,
    icd_RasterPos2d,
    icd_RasterPos2dv,
    icd_RasterPos2f,
    icd_RasterPos2fv,
    icd_RasterPos2i,
    icd_RasterPos2iv,
    icd_RasterPos2s,
    icd_RasterPos2sv,
    icd_RasterPos3d,
    icd_RasterPos3dv,
    icd_RasterPos3f,
    icd_RasterPos3fv,
    icd_RasterPos3i,
    icd_RasterPos3iv,
    icd_RasterPos3s,
    icd_RasterPos3sv,
    icd_RasterPos4d,
    icd_RasterPos4dv,
    icd_RasterPos4f,
    icd_RasterPos4fv,
    icd_RasterPos4i,
    icd_RasterPos4iv,
    icd_RasterPos4s,
    icd_RasterPos4sv,
    icd_Rectd,
    icd_Rectdv,
    icd_Rectf,
    icd_Rectfv,
    icd_Recti,
    icd_Rectiv,
    icd_Rects,
    icd_Rectsv,
    icd_TexCoord1d,
    icd_TexCoord1dv,
    icd_TexCoord1f,
    icd_TexCoord1fv,
    icd_TexCoord1i,
    icd_TexCoord1iv,
    icd_TexCoord1s,
    icd_TexCoord1sv,
    icd_TexCoord2d,
    icd_TexCoord2dv,
    icd_TexCoord2f,
    icd_TexCoord2fv,
    icd_TexCoord2i,
    icd_TexCoord2iv,
    icd_TexCoord2s,
    icd_TexCoord2sv,
    icd_TexCoord3d,
    icd_TexCoord3dv,
    icd_TexCoord3f,
    icd_TexCoord3fv,
    icd_TexCoord3i,
    icd_TexCoord3iv,
    icd_TexCoord3s,
    icd_TexCoord3sv,
    icd_TexCoord4d,
    icd_TexCoord4dv,
    icd_TexCoord4f,
    icd_TexCoord4fv,
    icd_TexCoord4i,
    icd_TexCoord4iv,
    icd_TexCoord4s,
    icd_TexCoord4sv,
    icd_Vertex2d,
    icd_Vertex2dv,
    icd_Vertex2f,
    icd_Vertex2fv,
    icd_Vertex2i,
    icd_Vertex2iv,
    icd_Vertex2s,
    icd_Vertex2sv,
    icd_Vertex3d,
    icd_Vertex3dv,
    icd_Vertex3f,
    icd_Vertex3fv,
    icd_Vertex3i,
    icd_Vertex3iv,
    icd_Vertex3s,
    icd_Vertex3sv,
    icd_Vertex4d,
    icd_Vertex4dv,
    icd_Vertex4f,
    icd_Vertex4fv,
    icd_Vertex4i,
    icd_Vertex4iv,
    icd_Vertex4s,
    icd_Vertex4sv,
    icd_ClipPlane,
    icd_ColorMaterial,
    icd_CullFace,
    icd_Fogf,
    icd_Fogfv,
    icd_Fogi,
    icd_Fogiv,
    icd_FrontFace,
    icd_Hint,
    icd_Lightf,
    icd_Lightfv,
    icd_Lighti,
    icd_Lightiv,
    icd_LightModelf,
    icd_LightModelfv,
    icd_LightModeli,
    icd_LightModeliv,
    icd_LineStipple,
    icd_LineWidth,
    icd_Materialf,
    icd_Materialfv,
    icd_Materiali,
    icd_Materialiv,
    icd_PointSize,
    icd_PolygonMode,
    icd_PolygonStipple,
    icd_Scissor,
    icd_ShadeModel,
    icd_TexParameterf,
    icd_TexParameterfv,
    icd_TexParameteri,
    icd_TexParameteriv,
    icd_TexImage1D,
    icd_TexImage2D,
    icd_TexEnvf,
    icd_TexEnvfv,
    icd_TexEnvi,
    icd_TexEnviv,
    icd_TexGend,
    icd_TexGendv,
    icd_TexGenf,
    icd_TexGenfv,
    icd_TexGeni,
    icd_TexGeniv,
    icd_FeedbackBuffer,
    icd_SelectBuffer,
    icd_RenderMode,
    icd_InitNames,
    icd_LoadName,
    icd_PassThrough,
    icd_PopName,
    icd_PushName,
    icd_DrawBuffer,
    icd_Clear,
    icd_ClearAccum,
    icd_ClearIndex,
    icd_ClearColor,
    icd_ClearStencil,
    icd_ClearDepth,
    icd_StencilMask,
    icd_ColorMask,
    icd_DepthMask,
    icd_IndexMask,
    icd_Accum,
    icd_Disable,
    icd_Enable,
    icd_Finish,
    icd_Flush,
    icd_PopAttrib,
    icd_PushAttrib,
    icd_Map1d,
    icd_Map1f,
    icd_Map2d,
    icd_Map2f,
    icd_MapGrid1d,
    icd_MapGrid1f,
    icd_MapGrid2d,
    icd_MapGrid2f,
    icd_EvalCoord1d,
    icd_EvalCoord1dv,
    icd_EvalCoord1f,
    icd_EvalCoord1fv,
    icd_EvalCoord2d,
    icd_EvalCoord2dv,
    icd_EvalCoord2f,
    icd_EvalCoord2fv,
    icd_EvalMesh1,
    icd_EvalPoint1,
    icd_EvalMesh2,
    icd_EvalPoint2,
    icd_AlphaFunc,
    icd_BlendFunc,
    icd_LogicOp,
    icd_StencilFunc,
    icd_StencilOp,
    icd_DepthFunc,
    icd_PixelZoom,
    icd_PixelTransferf,
    icd_PixelTransferi,
    icd_PixelStoref,
    icd_PixelStorei,
    icd_PixelMapfv,
    icd_PixelMapuiv,
    icd_PixelMapusv,
    icd_ReadBuffer,
    icd_CopyPixels,
    icd_ReadPixels,
    icd_DrawPixels,
    icd_GetBooleanv,
    icd_GetClipPlane,
    icd_GetDoublev,
    icd_GetError,
    icd_GetFloatv,
    icd_GetIntegerv,
    icd_GetLightfv,
    icd_GetLightiv,
    icd_GetMapdv,
    icd_GetMapfv,
    icd_GetMapiv,
    icd_GetMaterialfv,
    icd_GetMaterialiv,
    icd_GetPixelMapfv,
    icd_GetPixelMapuiv,
    icd_GetPixelMapusv,
    icd_GetPolygonStipple,
    icd_GetString,
    icd_GetTexEnvfv,
    icd_GetTexEnviv,
    icd_GetTexGendv,
    icd_GetTexGenfv,
    icd_GetTexGeniv,
    icd_GetTexImage,
    icd_GetTexParameterfv,
    icd_GetTexParameteriv,
    icd_GetTexLevelParameterfv,
    icd_GetTexLevelParameteriv,
    icd_IsEnabled,
    icd_IsList,
    icd_DepthRange,
    icd_Frustum,
    icd_LoadIdentity,
    icd_LoadMatrixf,
    icd_LoadMatrixd,
    icd_MatrixMode,
    icd_MultMatrixf,
    icd_MultMatrixd,
    icd_Ortho,
    icd_PopMatrix,
    icd_PushMatrix,
    icd_Rotated,
    icd_Rotatef,
    icd_Scaled,
    icd_Scalef,
    icd_Translated,
    icd_Translatef,
    icd_Viewport,
    icd_ArrayElement,
    icd_BindTexture,
    icd_ColorPointer,
    icd_DisableClientState,
    icd_DrawArrays,
    icd_DrawElements,
    icd_EdgeFlagPointer,
    icd_EnableClientState,
    icd_IndexPointer,
    icd_Indexub,
    icd_Indexubv,
    icd_InterleavedArrays,
    icd_NormalPointer,
    icd_PolygonOffset,
    icd_TexCoordPointer,
    icd_VertexPointer,
    icd_AreTexturesResident,
    icd_CopyTexImage1D,
    icd_CopyTexImage2D,
    icd_CopyTexSubImage1D,
    icd_CopyTexSubImage2D,
    icd_DeleteTextures,
    icd_GenTextures,
    icd_GetPointerv,
    icd_IsTexture,
    icd_PrioritizeTextures,
    icd_TexSubImage1D,
    icd_TexSubImage2D,
    icd_PopClientAttrib,
    icd_PushClientAttrib
} };

} // extern "C"
