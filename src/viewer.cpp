#include "tech_core/viewer.hpp"
#include <GLFW/glfw3.h>
#include <png.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace tech_core {
namespace {
class Viewer {
public:
    Viewer(Simulation& simulation,bool visible,const ViewOptions& options)
        :sim(simulation),snapshot(!visible),view_options(options) {
        glfwSetErrorCallback([](int code,const char* msg){std::cerr<<"GLFW "<<code<<": "<<msg<<'\n';});
        if(!glfwInit()) throw std::runtime_error("GLFW init failed; use --headless or scripts/run_software_viewer.sh");
        glfwWindowHint(GLFW_VISIBLE,visible?GLFW_TRUE:GLFW_FALSE);
        glfwWindowHint(GLFW_SAMPLES,4);
        window=glfwCreateWindow(1440,1000,"RoboMaster 2026 | Technology Core",nullptr,nullptr);
        if(!window) {glfwTerminate();throw std::runtime_error("Unable to create OpenGL window");}
        glfwMakeContextCurrent(window);glfwSwapInterval(visible?1:0);
        std::cout<<"OpenGL renderer: "<<glGetString(GL_RENDERER)<<'\n';
        mjv_defaultCamera(&camera);mjv_defaultOption(&option);mjv_defaultScene(&scene);mjr_defaultContext(&context);
        // The generator reserves group 2 for energy-unit visual meshes only.
        // Filtering the scene leaves opaque materials and contact physics intact.
        if(view_options.hide_unit) option.geomgroup[2]=0;
        mjv_makeScene(sim.model(),&scene,std::max(3000,sim.model()->ngeom+1000));mjr_makeContext(sim.model(),&context,mjFONTSCALE_150);
        camera.type=mjCAMERA_FREE;
        configure_camera();
        glfwSetWindowUserPointer(window,this);
        glfwSetKeyCallback(window,key_callback);
        glfwSetCursorPosCallback(window,mouse_callback);
        glfwSetScrollCallback(window,[](GLFWwindow* w,double,double dy){
            auto& v=*static_cast<Viewer*>(glfwGetWindowUserPointer(w));
            v.manual_camera=true;
            mjv_moveCamera(v.sim.model(),mjMOUSE_ZOOM,0,-.05*dy,&v.scene,&v.camera);
        });
    }
    ~Viewer(){mjr_freeContext(&context);mjv_freeScene(&scene);glfwDestroyWindow(window);glfwTerminate();}
    void render() {
        int width,height;glfwGetFramebufferSize(window,&width,&height);mjrRect rect{0,0,width,height};
        if(!manual_camera) configure_camera();
        mjv_updateScene(sim.model(),sim.data(),&option,nullptr,&camera,mjCAT_ALL,&scene);
        mjr_render(rect,&scene,&context);
        if(!view_options.overlay) return;
        std::ostringstream left,right;
        left<<"TECHNOLOGY CORE / RM 2026\nMuJoCo "<<mj_versionString()<<" | CPU physics\n\n"
            <<"Level "<<sim.config().level<<"   "<<(snapshot?"SNAPSHOT":demo?"AUTO":"MANUAL")<<(paused?" / PAUSED":"")
            <<"   t="<<std::fixed<<std::setprecision(2)<<sim.time()<<" s\n"
            <<"Selected: core "<<selected<<" | card "<<(sim.card_present()?"ON":"OFF")
            <<" | gripper "<<(sim.gripper_enabled()?"ON":"OFF")<<"\n";
        for(std::size_t i=0;i<sim.rules().core_count();++i) {
            auto o=sim.observation(i);
            left<<"\nCore "<<i<<": "<<stage_name(sim.rules().stage(i))
                <<"\n  insert "<<o.insertion*1000<<" / lift "<<o.lift*1000<<" mm"
                <<"\n  P "<<degrees(o.flip)<<" / Q "<<degrees(o.twist)<<" deg"
                <<"\n  target "<<sim.config().target_degrees[i]<<" deg"
                <<(sim.rules().safety_locked(i)?" | HOLD":"")<<"\n";
        }
        if(sim.rules().successful()) left<<"\nASSEMBLY CONFIRMED";
        if(sim.rules().failed()) left<<"\nFAILED: "<<sim.rules().failure_reason();
        right<<"Tab: select core\nI / K: insert / withdraw\nU / J: lift / lower\nO / L: P flip / return\nA / D: Q - / +\nEnter: confirm\nG: restart / stop demo\nX: release / grip\nC: interaction card\nSpace: pause\nR: reset\nMouse: orbit / pan\nWheel: zoom\nEsc: exit\n\nGuided energy unit\nInternal linkage is approximate";
        const auto l=left.str(),r=right.str();
        mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,rect,l.c_str(),nullptr,&context);
        mjr_overlay(mjFONT_NORMAL,mjGRID_TOPRIGHT,rect,r.c_str(),nullptr,&context);
    }
    void png(const std::string& path) {
        int width,height;glfwGetFramebufferSize(window,&width,&height);
        std::vector<unsigned char> rgb(static_cast<std::size_t>(width)*height*3);
        mjr_readPixels(rgb.data(),nullptr,mjrRect{0,0,width,height},&context);
        // libpng simplified API: negative stride flips MuJoCo's bottom-up pixels.
        png_image out{};out.version=PNG_IMAGE_VERSION;out.width=width;out.height=height;out.format=PNG_FORMAT_RGB;
        if(!png_image_write_to_file(&out,path.c_str(),0,rgb.data(),-width*3,nullptr)) {
            const std::string message=out.message;png_image_free(&out);
            throw std::runtime_error("PNG write failed: "+message);
        }
        png_image_free(&out);std::cout<<"Screenshot: "<<path<<'\n';
    }
    void manual_step(double dt) {
        auto c=sim.command(selected);
        auto down=[&](int key){return glfwGetKey(window,key)==GLFW_PRESS?1.0:0.0;};
        c.insertion+=(down(GLFW_KEY_I)-down(GLFW_KEY_K))*.05*dt;
        c.lift+=(down(GLFW_KEY_U)-down(GLFW_KEY_J))*.05*dt;
        c.flip+=(down(GLFW_KEY_O)-down(GLFW_KEY_L))*radians(45)*dt;
        c.twist+=(down(GLFW_KEY_D)-down(GLFW_KEY_A))*radians(35)*dt;
        sim.set_command(selected,c);
    }
    Simulation& sim;GLFWwindow* window=nullptr;bool demo=false,paused=false;
private:
    void configure_camera() {
        const auto& preset=view_options.view;
        camera.azimuth=205;camera.elevation=-17;
        if(preset=="iso") {
            camera.lookat[0]=-.16;camera.lookat[1]=sim.config().level==4?0:.15;
            camera.lookat[2]=.78;camera.distance=sim.config().level==4?2.25:1.95;
        } else if(preset=="marker" || preset=="unit") {
            const char* name=preset=="marker"?"core0_marker":"core0_grip";
            const int site=mj_name2id(sim.model(),mjOBJ_SITE,name);
            if(site<0) throw std::runtime_error(std::string("Inspection view requires site: ")+name);
            std::copy_n(sim.data()->site_xpos+3*site,3,camera.lookat);
            const int body=sim.model()->site_bodyid[site];
            const auto* rotation=sim.data()->xmat+9*body;
            // The local +X axis faces out of the inspected part. MuJoCo's
            // azimuth/elevation describe the viewing direction toward lookat.
            camera.azimuth=degrees(std::atan2(-rotation[3],-rotation[0]));
            camera.elevation=degrees(std::asin(std::clamp(-rotation[6],-1.0,1.0)));
            camera.distance=preset=="marker"?.20:.35;
            if(preset=="unit") {camera.azimuth+=25;camera.elevation-=17;}
        } else {
            const int body=mj_name2id(sim.model(),mjOBJ_BODY,"core0_base");
            if(body<0) throw std::runtime_error("Inspection view requires body: core0_base");
            std::copy_n(sim.data()->xpos+3*body,3,camera.lookat);
            camera.lookat[2]+=.20;camera.distance=1.0;
            camera.azimuth=preset=="rear"?0:preset=="side"?90:180;
            camera.elevation=preset=="top"?-90:0;
        }
        if(view_options.distance>0) camera.distance=view_options.distance;
    }
    static void key_callback(GLFWwindow* w,int key,int,int action,int) {
        if(action!=GLFW_PRESS) return;
        auto& v=*static_cast<Viewer*>(glfwGetWindowUserPointer(w));
        switch(key) {
            case GLFW_KEY_ESCAPE:glfwSetWindowShouldClose(w,1);break;
            case GLFW_KEY_SPACE:v.paused=!v.paused;break;
            case GLFW_KEY_TAB:v.selected=(v.selected+1)%v.sim.rules().core_count();break;
            case GLFW_KEY_G:v.demo=!v.demo;if(v.demo) {v.sim.reset();v.manual_camera=false;}break;
            case GLFW_KEY_R:v.sim.reset();v.manual_camera=false;break;
            case GLFW_KEY_C:v.sim.set_card_present(!v.sim.card_present());break;
            case GLFW_KEY_X:v.sim.set_gripper_enabled(!v.sim.gripper_enabled());break;
            case GLFW_KEY_ENTER:v.sim.confirm(v.selected);break;
        }
    }
    static void mouse_callback(GLFWwindow* w,double x,double y) {
        auto& v=*static_cast<Viewer*>(glfwGetWindowUserPointer(w));
        const double dx=x-v.mouse_x,dy=y-v.mouse_y;v.mouse_x=x;v.mouse_y=y;
        const bool left=glfwGetMouseButton(w,GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS;
        const bool right=glfwGetMouseButton(w,GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS;
        const bool middle=glfwGetMouseButton(w,GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS;
        if(!left&&!right&&!middle) return;
        v.manual_camera=true;
        int width,height;glfwGetWindowSize(w,&width,&height);if(height<=0) return;
        const bool shift=glfwGetKey(w,GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS;
        const int action=right?(shift?mjMOUSE_MOVE_H:mjMOUSE_MOVE_V):left?(shift?mjMOUSE_ROTATE_H:mjMOUSE_ROTATE_V):mjMOUSE_ZOOM;
        mjv_moveCamera(v.sim.model(),action,dx/height,dy/height,&v.scene,&v.camera);
    }
    bool snapshot=false;
    ViewOptions view_options;
    bool manual_camera=false;
    std::size_t selected=0;double mouse_x=0,mouse_y=0;
    mjvCamera camera{};mjvOption option{};mjvScene scene{};mjrContext context{};
};
}
void save_snapshot(Simulation& sim,const std::string& path,const ViewOptions& options) {
    Viewer viewer(sim,false,options);viewer.render();viewer.png(path);
}
void run_viewer(Simulation& sim,bool demo,double duration,const std::string& screenshot,const ViewOptions& options) {
    Viewer viewer(sim,true,options);viewer.demo=demo;
    while(!glfwWindowShouldClose(viewer.window) && (duration==0 || sim.time()<duration)) {
        const auto frame_start=std::chrono::steady_clock::now();
        glfwPollEvents();
        if(!viewer.paused) {
            const double end=sim.time()+1.0/60;
            while(sim.time()<end && (duration==0 || sim.time()<duration)) {
                if(viewer.demo) sim.drive_demo();else viewer.manual_step(sim.model()->opt.timestep);
                sim.step();
            }
        }
        viewer.render();glfwSwapBuffers(viewer.window);
        std::this_thread::sleep_until(frame_start+std::chrono::milliseconds(16));
    }
    if(!screenshot.empty()) {viewer.render();viewer.png(screenshot);}
}
} // namespace tech_core
