/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <iostream>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <fstream>
#include <string>

#ifdef WIN32
    #define OVR_OS_WIN32
#elif defined(__APPLE__)
    #define OVR_OS_MAC
#else
    #define OVR_OS_LINUX
    #include <X11/Xlib.h>
    #include <GL/glx.h>
#endif

#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>
#include <CAPI/CAPI_HSWDisplay.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

SDL_Window *win;
SDL_GLContext ctx;
int win_width, win_height;

unsigned int fbo, fb_tex, fb_depth;
int fb_width, fb_height;
int fb_tex_width, fb_tex_height;

ovrHmd hmd;
ovrSizei eyeres[2];
ovrEyeRenderDesc eye_rdesc[2];
ovrGLTexture fb_ovr_tex[2];
ovrGLConfig glcfg;
unsigned int distort_caps;
unsigned int hmd_caps;

BufferedPort<ImageOf<PixelRgb> > leftImagePort; // make a port for reading images
BufferedPort<ImageOf<PixelRgb> > rightImagePort; // make a port for reading images

int init(void);
void update_rtarg(int width, int height);
unsigned int next_pow2(unsigned int x);
void display(void);
void quat_to_matrix(const float *quat, float *mat);
void display(void);
void draw_scene(int);
void draw_box(float xsz, float ysz, float zsz, float norm_sign);
void cleanup(void);
int handle_event(SDL_Event *ev);
int key_event(int key, int state);
void reshape(int x, int y);
void toggle_hmd_fullscreen(void);

GLuint texture;

void InitializeIplToTexture(GLubyte *image)
{

  glGenTextures(1,&texture);
  glBindTexture(GL_TEXTURE_2D,texture);
  glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_DECAL);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);

  glTexImage2D(GL_TEXTURE_2D,0,2,640,480,0,GL_BGR,GL_UNSIGNED_BYTE,image);
}

void UpdateIplToTexture(IplImage *image)
{
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D,0,3,640,480,0,GL_RGB,GL_UNSIGNED_BYTE, image->imageData);
}


int main(int argc, char **argv)
{
    Network yarp; // set up yarp

    string line;
    string networkType;

    if( argc < 3 )
    {
        cout << "ERROR: Missing arguments" << endl;
        return false;
    }

    string param = argv[1];
    string fileName = argv[2];

    if( param.compare("--from") == 1 )
    {
        cout << "ERROR: [--from] parameter missing" << endl;
        return false;
    }    


    ifstream configFile(fileName.c_str());

    if(configFile.is_open())
    {
        while( getline(configFile, line) )
        {
            if( line.compare("[network_settings]") == 0 )
            {
                getline(configFile, line);
                networkType = line;                
            }
        }
        configFile.close();
    }

    cout << "Network type: " << networkType << endl;

    leftImagePort.open("/telepresence/leftEye:i"); // give the port a name
    rightImagePort.open("/telepresence/rightEye:i"); // give the port a name


    if( networkType.compare("local") == 0 )
    {
        cout << "Waiting for connections: /icub/cam/left to /telepresence/leftEye:i   and   /icub/cam/right to /telepresence/rightEye:i" << endl;

        while( !Network::isConnected("/icub/cam/left","/telepresence/leftEye:i") || !Network::isConnected("/icub/cam/right","/telepresence/rightEye:i"))
    	{}
//        while( !Network::isConnected("udp+mjpeg://icub/cam/left","/telepresence/leftEye:i") || !Network::isConnected("udp+mjpeg://icub/cam/right","/telepresence/rightEye:i"))
//    	{}
        cout << "Connections ready" << endl;
    }
    else if( networkType.compare("remote") == 0 )
    {        
        cout << "Waiting for connections: /gtw/telepresence/leftEye:o to /telepresence/leftEye:i   and  /gtw/telepresence/rightEye:o to /telepresence/rightEye:i" << endl;

        while( !Network::isConnected("/gtw/telepresence/leftEye:o","/telepresence/leftEye:i") || !Network::isConnected("/gtw/telepresence/rightEye:o","/telepresence/rightEye:i"))
    	{}

        cout << "Connections ready" << endl;
    }
    

    if(init() == -1)
    {
        return 1;
    }

    toggle_hmd_fullscreen();

    for(;;)
    {
/*        SDL_Event ev;
        while(SDL_PollEvent(&ev))
        {
            if(handle_event(&ev) == -1)
            {
                goto done;
            }
        }
*/
        display();
    }

//    done:
    cleanup();

    printf("...done\n");

    return 0;
}



int init(void)
{
    int i, x, y;
    unsigned int flags;

    /* libovr must be initialized before we create the OpenGL context */
    ovr_Initialize();

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);

    x = y = SDL_WINDOWPOS_UNDEFINED;
    flags = SDL_WINDOW_OPENGL;
    if(!(win = SDL_CreateWindow("press 'f' to move to the HMD", x, y, 1024, 640, flags)))
    {
        fprintf(stderr, "failed to create window\n");
        return -1;
    }
    
    if(!(ctx = SDL_GL_CreateContext(win)))
    {
        fprintf(stderr, "failed to create OpenGL context\n");
        return -1;
    }

    glewInit();

    if(!(hmd = ovrHmd_Create(0)))
    {
        fprintf(stderr, "failed to open Oculus HMD, falling back to virtual debug HMD\n");
        if(!(hmd = ovrHmd_CreateDebug(ovrHmd_DK2)))
        {
            fprintf(stderr, "failed to create virtual debug HMD\n");
            return -1;
        }
    }
    
    printf("initialized HMD: %s - %s\n", hmd->Manufacturer, hmd->ProductName);

    /* resize our window to match the HMD resolution */
    SDL_SetWindowSize(win, hmd->Resolution.w, hmd->Resolution.h);
    SDL_SetWindowPosition(win, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    win_width = hmd->Resolution.w;
    win_height = hmd->Resolution.h;

    /* enable position and rotation tracking */
    ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);
    /* retrieve the optimal render target resolution for each eye */
    eyeres[0] = ovrHmd_GetFovTextureSize(hmd, ovrEye_Left, hmd->DefaultEyeFov[0], 1.0);
    eyeres[1] = ovrHmd_GetFovTextureSize(hmd, ovrEye_Right, hmd->DefaultEyeFov[1], 1.0);

    /* and create a single render target texture to encompass both eyes */
    fb_width = eyeres[0].w + eyeres[1].w;
    fb_height = eyeres[0].h > eyeres[1].h ? eyeres[0].h : eyeres[1].h;

    fb_width = 640;
    fb_height = 480;

    update_rtarg(fb_width, fb_height);

    /* fill in the ovrGLTexture structures that describe our render target texture */
    for(i=0; i<2; i++)
    {
        fb_ovr_tex[i].OGL.Header.API = ovrRenderAPI_OpenGL;
        fb_ovr_tex[i].OGL.Header.TextureSize.w = fb_tex_width;
        fb_ovr_tex[i].OGL.Header.TextureSize.h = fb_tex_height;
        /* this next field is the only one that differs between the two eyes */
        fb_ovr_tex[i].OGL.Header.RenderViewport.Pos.x = i == 0 ? 0 : fb_width / 2.0;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Pos.y = 0;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Size.w = fb_width / 2.0;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Size.h = fb_height;
        fb_ovr_tex[i].OGL.TexId = fb_tex; /* both eyes will use the same texture id */
    }

    /* fill in the ovrGLConfig structure needed by the SDK to draw our stereo pair
    * to the actual HMD display (SDK-distortion mode)
    */
    memset(&glcfg, 0, sizeof glcfg);
    glcfg.OGL.Header.API = ovrRenderAPI_OpenGL;
    glcfg.OGL.Header.BackBufferSize.w = win_width;
    glcfg.OGL.Header.BackBufferSize.h = win_height;
    glcfg.OGL.Header.Multisample = 1;

    #ifdef OVR_OS_WIN32
        glcfg.OGL.Window = GetActiveWindow();
        glcfg.OGL.DC = wglGetCurrentDC();
    #elif defined(OVR_OS_LINUX)
        glcfg.OGL.Disp = glXGetCurrentDisplay();
    #endif

    if(hmd->HmdCaps & ovrHmdCap_ExtendDesktop)
    {
        printf("running in \"extended desktop\" mode\n");
    }
    else
    {
        /* to sucessfully draw to the HMD display in "direct-hmd" mode, we have to
        * call ovrHmd_AttachToWindow
        * XXX: this doesn't work properly yet due to bugs in the oculus 0.4.1 sdk/driver
        */
        #ifdef WIN32
            ovrHmd_AttachToWindow(hmd, glcfg.OGL.Window, 0, 0);
        #elif defined(OVR_OS_LINUX)
            ovrHmd_AttachToWindow(hmd, (void*)glXGetCurrentDrawable(), 0, 0);
        #endif
            printf("running in \"direct-hmd\" mode\n");
    }


    /* enable low-persistence display and dynamic prediction for lattency compensation */
    hmd_caps = ovrHmdCap_LowPersistence | ovrHmdCap_DynamicPrediction;
    ovrHmd_SetEnabledCaps(hmd, hmd_caps);

    /* configure SDK-rendering and enable chromatic abberation correction, vignetting, and
    * timewrap, which shifts the image before drawing to counter any lattency between the call
    * to ovrHmd_GetEyePose and ovrHmd_EndFrame.
    */

    distort_caps = ovrDistortionCap_Chromatic | ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive;

    if(!ovrHmd_ConfigureRendering(hmd, &glcfg.Config, distort_caps, hmd->DefaultEyeFov, eye_rdesc))
    {
        fprintf(stderr, "failed to configure distortion renderer\n");
    }

    /* disable the retarded "health and safety warning" */
    ovrhmd_EnableHSWDisplaySDKRender(hmd, 0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_NORMALIZE);
    glClearColor(0.1, 0.1, 0.1, 1);


    GLubyte *texture_data = new GLubyte[640*480]();
    InitializeIplToTexture(texture_data);
    
    return 0;
} 


/* update_rtarg creates (and/or resizes) the render target used to draw the two stero views */
void update_rtarg(int width, int height)
{
    if(!fbo)
    {
        // if fbo does not exist, then nothing does... create every opengl object
        glGenFramebuffers(1, &fbo);
        glGenTextures(1, &fb_tex);
        glGenRenderbuffers(1, &fb_depth);

        glBindTexture(GL_TEXTURE_2D, fb_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    /* calculate the next power of two in both dimensions and use that as a texture size */
//    fb_tex_width = next_pow2(width);
//    fb_tex_height = next_pow2(height);
    fb_tex_width = 640;
    fb_tex_height = 480;

    /* create and attach the texture that will be used as a color buffer */
    glBindTexture(GL_TEXTURE_2D, fb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, fb_tex_width, fb_tex_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fb_tex, 0);

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        fprintf(stderr, "incomplete framebuffer!\n");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    printf("created render target: %dx%d (texture size: %dx%d)\n", width, height, fb_tex_width, fb_tex_height);
}

unsigned int next_pow2(unsigned int x)
{
    x -= 1;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    return x + 1;
} 

void display(void)
{
    int i;
    ovrMatrix4f proj;
    ovrPosef pose[2];

    /* the drawing starts with a call to ovrHmd_BeginFrame */
    ovrHmd_BeginFrame(hmd, 0);

    /* start drawing onto our texture render target */
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* for each eye ... */
    for(i=0; i<2; i++)
    {
        ovrEyeType eye = hmd->EyeRenderOrder[i];

        /* -- viewport transformation --
        * setup the viewport to draw in the left half of the framebuffer when we're
        * rendering the left eye's view (0, 0, width/2, height), and in the right half
        * of the framebuffer for the right eye's view (width/2, 0, width/2, height)
        */
        glViewport(eye == ovrEye_Left ? 0 : fb_width / 2, 0, fb_width / 2, fb_height);

        /* -- projection transformation --
        * we'll just have to use the projection matrix supplied by the oculus SDK for this eye
        * note that libovr matrices are the transpose of what OpenGL expects, so we have to
        * use glLoadTransposeMatrixf instead of glLoadMatrixf to load it.
        */
        proj = ovrMatrix4f_Projection(hmd->DefaultEyeFov[eye], 0.5, 500.0, 1);
        glMatrixMode(GL_PROJECTION);
        glLoadTransposeMatrixf(proj.M[0]);

        /* -- view/camera transformation --
        * we need to construct a view matrix by combining all the information provided by the oculus
        * SDK, about the position and orientation of the user's head in the world.
        */
        /* TODO: use ovrHmd_GetEyePoses out of the loop instead */
        pose[eye] = ovrHmd_GetHmdPosePerEye(hmd, eye);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(eye_rdesc[eye].HmdToEyeViewOffset.x,
        eye_rdesc[eye].HmdToEyeViewOffset.y,
        eye_rdesc[eye].HmdToEyeViewOffset.z);
        /* move the camera to the eye level of the user */
        glTranslatef(0, -ovrHmd_GetFloat(hmd, OVR_KEY_EYE_HEIGHT, 1.65), 0);

        /* finally draw the scene for this eye */
		if( i == 1 )
			draw_scene(1);
//	        draw_scene_leftEye();
		else
			draw_scene(0);
//	        draw_scene_rightEye();
    }

    /* after drawing both eyes into the texture render target, revert to drawing directly to the
    * display, and we call ovrHmd_EndFrame, to let the Oculus SDK draw both images properly
    * compensated for lens distortion and chromatic abberation onto the HMD screen.
    */
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    ovrHmd_EndFrame(hmd, pose, &fb_ovr_tex[0].Texture);

    /* workaround for the oculus sdk distortion renderer bug, which uses a shader
    * program, and doesn't restore the original binding when it's done.
    */
    glUseProgram(0);

    assert(glGetError() == GL_NO_ERROR);
}


/* convert a quaternion to a rotation matrix */
void quat_to_matrix(const float *quat, float *mat)
{
    mat[0] = 1.0 - 2.0 * quat[1] * quat[1] - 2.0 * quat[2] * quat[2];
    mat[4] = 2.0 * quat[0] * quat[1] + 2.0 * quat[3] * quat[2];
    mat[8] = 2.0 * quat[2] * quat[0] - 2.0 * quat[3] * quat[1];
    mat[12] = 0.0f;

    mat[1] = 2.0 * quat[0] * quat[1] - 2.0 * quat[3] * quat[2];
    mat[5] = 1.0 - 2.0 * quat[0]*quat[0] - 2.0 * quat[2]*quat[2];
    mat[9] = 2.0 * quat[1] * quat[2] + 2.0 * quat[3] * quat[0];
    mat[13] = 0.0f;

    mat[2] = 2.0 * quat[2] * quat[0] + 2.0 * quat[3] * quat[1];
    mat[6] = 2.0 * quat[1] * quat[2] - 2.0 * quat[3] * quat[0];
    mat[10] = 1.0 - 2.0 * quat[0]*quat[0] - 2.0 * quat[1]*quat[1];
    mat[14] = 0.0f;

    mat[3] = mat[7] = mat[11] = 0.0f;
    mat[15] = 1.0f;
} 

void draw_scene(int eye_id)
{
    int i;
    float grey[] = {0.8, 0.8, 0.8, 1};
    float lpos[][4] = {{-8, 2, 10, 1},{0, 15, 0, 1}};
    float lcol[][4] = {{0.8, 0.8, 0.8, 1},{0.4, 0.3, 0.3, 1}};

    for(i=0; i<2; i++)
    {
        glLightfv(GL_LIGHT0 + i, GL_POSITION, lpos[i]);
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, lcol[i]);
    }

    glMatrixMode(GL_MODELVIEW);

    glPushMatrix();
    glTranslatef(0, 1.5, 0);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, grey);

	ImageOf<PixelRgb> *eyeImage = NULL; // read an image

	if( eye_id == 0 )
		eyeImage = rightImagePort.read(); // read an image
	else
		eyeImage = leftImagePort.read(); // read an image

	IplImage *iplYarpImage = (IplImage*)eyeImage->getIplImage();
	cvFlip(iplYarpImage, iplYarpImage,0);
	cvFlip(iplYarpImage, iplYarpImage,1);

    UpdateIplToTexture(iplYarpImage);

    glEnable(GL_TEXTURE_2D);
    draw_box(30, 38, 30, -1.0);
    glDisable(GL_TEXTURE_2D);
    glPopMatrix();
}

void draw_box(float xsz, float ysz, float zsz, float norm_sign)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glScalef(xsz * 0.5, ysz * 0.5, zsz * 0.5);

    if(norm_sign < 0.0)
    {
        glFrontFace(GL_CW);
    }

    glBegin(GL_QUADS);
    glNormal3f(0, 0, -1 * norm_sign);
    glTexCoord2f(0, 0); glVertex3f(1, -1, -1);
    glTexCoord2f(1, 0); glVertex3f(-1, -1, -1);
    glTexCoord2f(1, 1); glVertex3f(-1, 1, -1);
    glTexCoord2f(0, 1); glVertex3f(1, 1, -1);
    glEnd();
    glFrontFace(GL_CCW);
    glPopMatrix();
}

void cleanup(void)
{
    if(hmd)
    {
        ovrHmd_Destroy(hmd);
    }

    ovr_Shutdown();
    SDL_Quit();
}


int handle_event(SDL_Event *ev)
{
    switch(ev->type)
    {
        case SDL_QUIT:
            return -1;

        case SDL_KEYDOWN:
        case SDL_KEYUP:
            if(key_event(ev->key.keysym.sym, ev->key.state == SDL_PRESSED) == -1)
            {
                return -1;
            }
            break;

        case SDL_WINDOWEVENT:
            if(ev->window.event == SDL_WINDOWEVENT_RESIZED)
            {
                reshape(ev->window.data1, ev->window.data2);
            }
            break;

        default:
            break;
    }

    return 0;
}

int key_event(int key, int state)
{
    if(state)
    {
        switch(key)
        {
            case 27:
                return -1;

            case ' ':
            case 'r':
                /* allow the user to recenter by pressing space */
                ovrHmd_RecenterPose(hmd);
                break;

            case 'f':
                /* press f to move the window to the HMD */
                toggle_hmd_fullscreen();
                break;

            case 'v':
                distort_caps ^= ovrDistortionCap_Vignette;
                printf("Vignette: %s\n", distort_caps & ovrDistortionCap_Vignette ? "on" : "off");
                ovrHmd_ConfigureRendering(hmd, &glcfg.Config, distort_caps, hmd->DefaultEyeFov, eye_rdesc);
                break;

            case 't':
                distort_caps ^= ovrDistortionCap_TimeWarp;
                printf("Time-warp: %s\n", distort_caps & ovrDistortionCap_TimeWarp ? "on" : "off");
                ovrHmd_ConfigureRendering(hmd, &glcfg.Config, distort_caps, hmd->DefaultEyeFov, eye_rdesc);
                break;

            case 'o':
                distort_caps ^= ovrDistortionCap_Overdrive;
                printf("OLED over-drive: %s\n", distort_caps & ovrDistortionCap_Overdrive ? "on" : "off");
                ovrHmd_ConfigureRendering(hmd, &glcfg.Config, distort_caps, hmd->DefaultEyeFov, eye_rdesc);
                break;

            case 'l':
                hmd_caps ^= ovrHmdCap_LowPersistence;
                printf("Low-persistence display: %s\n", hmd_caps & ovrHmdCap_LowPersistence ? "on" : "off");
                ovrHmd_SetEnabledCaps(hmd, hmd_caps);
                break;

            default:
                break;
        }
    }
    return 0;
}


void reshape(int x, int y)
{
    win_width = x;
    win_height = y;
} 

void toggle_hmd_fullscreen(void)
{
    static int fullscr, prev_x, prev_y;
    fullscr = !fullscr;

    if(fullscr)
    {
        /* going fullscreen on the rift. save current window position, and move it
        * to the rift's part of the desktop before going fullscreen
        */
        SDL_GetWindowPosition(win, &prev_x, &prev_y);
        SDL_SetWindowPosition(win, hmd->WindowsPos.x, hmd->WindowsPos.y);
        SDL_SetWindowFullscreen(win, SDL_WINDOW_FULLSCREEN_DESKTOP);

        #ifdef OVR_OS_LINUX
            /* on linux for now we have to deal with screen rotation during rendering. The docs are promoting
            * not rotating the DK2 screen globally
            */
            glcfg.OGL.Header.BackBufferSize.w = hmd->Resolution.h;
            glcfg.OGL.Header.BackBufferSize.h = hmd->Resolution.w;

            distort_caps |= ovrDistortionCap_LinuxDevFullscreen;
            ovrHmd_ConfigureRendering(hmd, &glcfg.Config, distort_caps, hmd->DefaultEyeFov, eye_rdesc);
        #endif
    }
    else
    {
        /* return to windowed mode and move the window back to its original position */
        SDL_SetWindowFullscreen(win, 0);
        SDL_SetWindowPosition(win, prev_x, prev_y);

        #ifdef OVR_OS_LINUX
            glcfg.OGL.Header.BackBufferSize = hmd->Resolution;

            distort_caps &= ~ovrDistortionCap_LinuxDevFullscreen;
            ovrHmd_ConfigureRendering(hmd, &glcfg.Config, distort_caps, hmd->DefaultEyeFov, eye_rdesc);
        #endif
    }
} 

