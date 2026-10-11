#include "tech_core/planning.hpp"
#include <GL/osmesa.h>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace tech_core;
using namespace tech_core::planning;
using Row = std::map<std::string, std::string>;

std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> out; std::string field; bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') { field += c; ++i; }
            else quoted = !quoted;
        } else if (c == ',' && !quoted) { out.push_back(field); field.clear(); }
        else if (c != '\r') field += c;
    }
    if (quoted) throw std::runtime_error("Unterminated CSV field");
    out.push_back(field); return out;
}
std::vector<Row> read_csv(const std::filesystem::path& path) {
    std::ifstream in(path); if (!in) throw std::runtime_error("Cannot open " + path.string());
    std::string line; std::getline(in, line); const auto header = fields(line);
    std::vector<Row> rows;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto values = fields(line); if (values.size() != header.size()) throw std::runtime_error("Invalid CSV row");
        Row row; for (std::size_t i = 0; i < values.size(); ++i) row[header[i]] = values[i];
        rows.push_back(std::move(row));
    }
    return rows;
}
double number(const Row& r, const std::string& key) { return std::stod(r.at(key)); }
Phase phase(const std::string& name) {
    for (int i = 0; i < 6; ++i) if (name == phase_name(static_cast<Phase>(i))) return static_cast<Phase>(i);
    throw std::runtime_error("Invalid phase");
}


struct Case {
    Scenario scenario;Row run;std::vector<Knot> knots;
    bool success=false;double duration=0,positive_time=0;
};
std::string short_reason(const std::string& reason) {
    if(reason.find("collision")!=std::string::npos) return "Collision / clearance rejection";
    if(reason.find("ik_")!=std::string::npos) return "IK did not find a complete path";
    if(reason.find("front_plane")!=std::string::npos)return "Initial front-plane rejection";
    return "No validated path in search budget";
}
int main(int argc,char** argv) {
  try {
    if(argc!=7)throw std::runtime_error("Usage: renderer DIRECTORY WIDTH HEIGHT FPS STILL_PAGE_OR_MINUS_ONE STILL_TIME");
    const std::filesystem::path directory=argv[1];const int width=std::stoi(argv[2]),height=std::stoi(argv[3]),fps=std::stoi(argv[4]),still_page=std::stoi(argv[5]);
    const double still_time=std::stod(argv[6]);
    if(width<1600||height<900||fps<1||still_page>4||!std::isfinite(still_time)||still_time<0)throw std::runtime_error("Invalid render arguments");
    const auto scenarios=read_csv(directory/"scenarios.csv"),runs=read_csv(directory/"runs.csv");
    if(scenarios.size()!=40||runs.size()!=40)throw std::runtime_error("Montage requires all 40 requested cases");
    std::vector<Case> cases;PlanningScene sim;int successes=0;std::size_t total_samples=0;
    for(const auto& r:scenarios) {
      Case c;auto& s=c.scenario;s.id=r.at("case");s.split=r.at("split");
      s.core={number(r,"core_x"),number(r,"core_y"),number(r,"core_z"),number(r,"core_theta_degrees"),number(r,"core_phi_degrees"),number(r,"core_alpha_degrees")};
      s.target_degrees=number(r,"target_degrees");s.q_sweep=r.at("q_sweep")=="1";
      s.assembly_translation={number(r,"assembly_translation_x"),number(r,"assembly_translation_y"),number(r,"assembly_translation_z")};
      s.assembly_yaw=number(r,"assembly_yaw_radians");s.arm_base_z=number(r,"arm_base_z");
      for(int j=0;j<7;++j)s.start[j]=number(r,"start_q"+std::to_string(j));
      for(const auto& run:runs)if(run.at("case")==s.id)c.run=run;
      if(c.run.empty())throw std::runtime_error("Missing run "+s.id);
      c.success=c.run.at("success")=="1";
      if(c.success) {
        if(c.run.at("dense_recheck_valid")!="1"||!s.q_sweep)throw std::runtime_error("Missing Q sweep validation");
        for(const auto& row:read_csv(directory/c.run.at("trajectory"))) {
          Knot k;k.time=number(row,"time");k.phase=phase(row.at("phase"));
          k.mechanism={number(row,"insertion"),number(row,"lift"),number(row,"flip"),number(row,"twist")};
          for(int j=0;j<7;++j)k.q[j]=number(row,"q"+std::to_string(j));
          k.target.position={number(row,"target_x"),number(row,"target_y"),number(row,"target_z")};
          k.target.direction={number(row,"direction_x"),number(row,"direction_y"),number(row,"direction_z")};
          c.knots.push_back(k);
          if(std::abs(k.mechanism.twist-pi/2)<1e-9)c.positive_time=k.time;
        }
        PlanOptions options;options.motion_speed_scale=number(c.run,"motion_speed_scale");options.validation_joint_step*=.5;options.validation_min_substeps*=2;
        sim.configure(s);const auto valid=validate_trajectory(sim,c.knots,options);
        if(!valid.valid)throw std::runtime_error(s.id+" dense recheck failed: "+valid.reason);
        c.duration=c.knots.back().time;++successes;total_samples+=valid.samples;
        std::cerr<<"DENSE_RECHECK "<<s.id<<" samples="<<valid.samples<<'\n';
      }
      cases.push_back(std::move(c));
    }
    std::cerr<<"VALIDATION_COMPLETE successes="<<successes<<" cases=40 samples="<<total_samples<<'\n';
    OSMesaContext os=OSMesaCreateContextExt(OSMESA_RGBA,24,8,0,nullptr);
    std::vector<unsigned char> framebuffer(std::size_t(width)*height*4);
    if(!os||!OSMesaMakeCurrent(os,framebuffer.data(),GL_UNSIGNED_BYTE,width,height))throw std::runtime_error("OSMesa failed");
    const std::string renderer=reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    std::cerr<<"OPENGL_RENDERER "<<renderer<<'\n';
    if(renderer.find("llvmpipe")==std::string::npos&&renderer.find("softpipe")==std::string::npos)throw std::runtime_error("Expected CPU software renderer");
    auto* model=sim.model();model->vis.global.offwidth=width;model->vis.global.offheight=height;model->vis.quality.offsamples=1;
    mjvOption option;mjv_defaultOption(&option);for(int i=0;i<mjNGROUP;++i){option.geomgroup[i]=i<=2;option.sitegroup[i]=0;}
    mjvScene scene;mjv_defaultScene(&scene);mjv_makeScene(model,&scene,model->ngeom+100);
    scene.flags[mjRND_REFLECTION]=0;scene.flags[mjRND_SHADOW]=0;
    mjrContext context;mjr_defaultContext(&context);mjr_makeContext(model,&context,mjFONTSCALE_100);mjr_setBuffer(mjFB_OFFSCREEN,&context);
    mjvCamera camera;mjv_defaultCamera(&camera);camera.lookat[0]=.19;camera.lookat[1]=0;camera.lookat[2]=.29;
    camera.distance=1.50;camera.azimuth=315;camera.elevation=-22;
    const mjrRect full{0,0,width,height};const int tile_width=width/4,tile_height=(height-120)/2;
    std::vector<unsigned char> rgb(std::size_t(width)*height*3);std::size_t emitted=0;
    for(int page=0;page<5;++page) {
      if(still_page>=0&&page!=still_page)continue;
      double duration=6;for(int i=page*8;i<(page+1)*8;++i)duration=std::max(duration,cases[i].duration+2);
      const int frames=still_page>=0?1:static_cast<int>(std::ceil(duration*fps));
      for(int frame=0;frame<frames;++frame) {
        const double t=still_page>=0?still_time:double(frame)/fps;
        mjr_rectangle(full,.04f,.055f,.075f,1);
        std::ostringstream title,footer;
        title<<"40 CORE POSES  |  LEVEL 3  |  Q: 0 > +90 > -90 deg\n"
             <<"Page "<<page+1<<" / 5   |   "<<successes<<" / 40 validated   |   Motion 4x   Playback 1x   Base Z -100 mm";
        mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,{10,height-74,width-20,70},title.str().c_str(),nullptr,&context);
        for(int local=0;local<8;++local) {
          auto& c=cases[page*8+local];sim.configure(c.scenario);
          const double now=std::min(t,c.duration);MechanismState mechanism{};Phase phase_now=Phase::Approach;
          if(c.success) {
            auto end=std::lower_bound(c.knots.begin()+1,c.knots.end(),now,[](const Knot& k,double value){return k.time<value;});
            if(end==c.knots.end())end=c.knots.end()-1;
            const auto& a=*(end-1);const auto& b=*end;phase_now=b.phase;
            const double u=std::clamp((now-a.time)/(b.time-a.time),0.,1.),blend=u*u*(3-2*u);
            const auto mix=[blend](double x,double y){return x+blend*(y-x);};
            mechanism={mix(a.mechanism.insertion,b.mechanism.insertion),mix(a.mechanism.lift,b.mechanism.lift),mix(a.mechanism.flip,b.mechanism.flip),mix(a.mechanism.twist,b.mechanism.twist)};
            sim.set_state((1-blend)*a.q+blend*b.q,mechanism);
          }
          const int x=(local%4)*tile_width+4,y=42+(1-local/4)*tile_height+3;
          const mjrRect tile{x,y,tile_width-8,tile_height-6},view{x,y+51,tile.width,tile.height-111};
          mjr_rectangle(tile,c.success?.045f:.16f,c.success?.09f:.045f,c.success?.10f:.055f,1);
          mjv_updateScene(model,sim.data(),&option,nullptr,&camera,mjCAT_ALL,&scene);mjr_render(view,&scene,&context);
          std::ostringstream pose,status;
          pose<<c.scenario.id<<"  "<<(c.success?"VALIDATED":"REJECTED - STATIC")<<"\n"<<std::fixed<<std::setprecision(0)
              <<"XYZ "<<c.scenario.core.x*1000<<","<<c.scenario.core.y*1000<<","<<c.scenario.core.z*1000<<" mm  YPR "
              <<c.scenario.core.theta<<","<<c.scenario.core.phi-90<<","<<c.scenario.core.alpha<<" deg";
          mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,{x,y+tile.height-58,tile.width,57},pose.str().c_str(),nullptr,&context);
          if(c.success) {
            status<<std::fixed<<std::setprecision(1)<<"t "<<now<<"/"<<c.duration<<"s  Q "<<mechanism.twist*180/pi<<" deg\n"
                  <<(now>=c.duration?"COMPLETE":phase_name(phase_now))<<(now>=c.positive_time?"  |  +90 reached":"  |  +90 pending");
          }else status<<"NOT EXECUTED\n"<<short_reason(c.run.at("reason"));
          mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,{x,y+1,tile.width,49},status.str().c_str(),nullptr,&context);
        }
        footer<<"All 40 requested poses shown. Red panels: rejected paths stay still. Fixed-grasp kinematic simulation.";
        mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,{10,3,width-20,35},footer.str().c_str(),nullptr,&context);
        mjr_readPixels(rgb.data(),nullptr,full,&context);
        if(std::fwrite(rgb.data(),1,rgb.size(),stdout)!=rgb.size())throw std::runtime_error("Encoder pipe closed");
        ++emitted;
        if(frame%(fps*2)==0||frame+1==frames)std::cerr<<"PAGE "<<page+1<<" FRAME "<<frame+1<<'/'<<frames<<" time="<<t<<'\n';
      }
    }
    std::cerr<<"FRAMES_COMPLETE "<<emitted<<'\n';std::fflush(stdout);
    mjr_freeContext(&context);mjv_freeScene(&scene);OSMesaDestroyContext(os);return 0;
  }catch(const std::exception& e){std::cerr<<"ERROR "<<e.what()<<'\n';return 1;}
}
