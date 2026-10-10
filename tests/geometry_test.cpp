#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Vec=std::array<double,3>;
constexpr double pi=3.14159265358979323846;
constexpr double vertex_tolerance=2e-7; // MuJoCo stores mesh coordinates as float32.
void require(bool condition,const std::string& message) {
    if(!condition) throw std::runtime_error(message);
}
void near(double actual,double expected,double tolerance,const std::string& message) {
    require(std::isfinite(actual)&&std::abs(actual-expected)<=tolerance,
            message+": expected "+std::to_string(expected)+", got "+std::to_string(actual));
}
void near(const Vec& actual,const Vec& expected,double tolerance,const std::string& message) {
    for(int axis=0;axis<3;++axis) near(actual[axis],expected[axis],tolerance,message+" axis "+std::to_string(axis));
}
int id(const mjModel* m,mjtObj type,const std::string& name) {
    const int value=mj_name2id(m,type,name.c_str());
    require(value>=0,"Missing actual model object "+name);
    return value;
}
Vec rotate(const mjtNum* matrix,const Vec& point) {
    Vec result{};mju_mulMatVec3(result.data(),matrix,point.data());return result;
}
Vec add(const Vec& a,const Vec& b) {return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
struct Triangle {std::array<int,3> vertices{},normals{};};
struct Obj {std::string name;std::vector<Vec> vertices,normals;std::vector<Triangle> triangles;};
Obj read_obj(const std::filesystem::path& path) {
    Obj obj;obj.name=path.stem().string();
    std::ifstream input(path);require(input.good(),"Cannot read "+path.string());
    std::string line;
    while(std::getline(input,line)) {
        std::istringstream fields(line);std::string type;fields>>type;
        if(type=="v"||type=="vn") {
            Vec value{};fields>>value[0]>>value[1]>>value[2];
            (type=="v"?obj.vertices:obj.normals).push_back(value);
        } else if(type=="f") {
            Triangle triangle;
            for(int corner=0;corner<3;++corner) {
                std::string token;fields>>token;
                const auto split=token.find("//");
                require(split!=std::string::npos,"Explicit source corner normal required");
                triangle.vertices[corner]=std::stoi(token.substr(0,split))-1;
                triangle.normals[corner]=std::stoi(token.substr(split+2))-1;
            }
            std::string extra;require(!(fields>>extra),"Source must be triangulated");
            obj.triangles.push_back(triangle);
        }
    }
    require(!obj.vertices.empty()&&!obj.triangles.empty(),"Empty source OBJ "+obj.name);
    return obj;
}
struct Scene {
    std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model{nullptr,mj_deleteModel};
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data{nullptr,mj_deleteData};
    Scene() {
        char error[2048]{};
        model.reset(mj_loadXML(TECH_CORE_MODEL_PATH,nullptr,error,sizeof(error)));
        require(model!=nullptr,std::string("MJCF load failed: ")+error);
        data.reset(mj_makeData(model.get()));
        for(int core=0;core<2;++core) {
            const int base=id(model.get(),mjOBJ_BODY,"core"+std::to_string(core)+"_base");
            const int mocap=model->body_mocapid[base];
            data->mocap_pos[3*mocap]=0;
            data->mocap_pos[3*mocap+1]=2*core;
            data->mocap_pos[3*mocap+2]=1;
            data->mocap_quat[4*mocap]=1;
            mju_zero(data->mocap_quat+4*mocap+1,3);
        }
        mj_forward(model.get(),data.get());
    }
    void joint(int core,const std::string& name,double value) {
        const int j=id(model.get(),mjOBJ_JOINT,"core"+std::to_string(core)+"_"+name);
        data->qpos[model->jnt_qposadr[j]]=value;
    }
};

Vec mesh_vertex(const mjModel* m,int mesh,int vertex) {
    const float* v=m->mesh_vert+3*(m->mesh_vertadr[mesh]+vertex);
    return {v[0],v[1],v[2]};
}
Vec world_vertex(const Scene& scene,int geom,int vertex) {
    const auto* m=scene.model.get();const auto* d=scene.data.get();
    const auto point=rotate(d->geom_xmat+9*geom,mesh_vertex(m,m->geom_dataid[geom],vertex));
    return add(point,{d->geom_xpos[3*geom],d->geom_xpos[3*geom+1],d->geom_xpos[3*geom+2]});
}

void complete_geometry_roundtrip(Scene& scene,const std::vector<Obj>& objects) {
    const auto* m=scene.model.get();
    std::size_t total_triangles=0,total_vertices=0;
    for(const auto& obj:objects) {
        const int mesh=id(m,mjOBJ_MESH,"hkust_"+obj.name);
        require(m->mesh_vertnum[mesh]==static_cast<int>(obj.vertices.size()),"Vertex loss "+obj.name);
        require(m->mesh_facenum[mesh]==static_cast<int>(obj.triangles.size()),"Triangle loss "+obj.name);
        mjtNum rotation[9];mju_quat2Mat(rotation,m->mesh_quat+4*mesh);
        const Vec translation={m->mesh_pos[3*mesh],m->mesh_pos[3*mesh+1],m->mesh_pos[3*mesh+2]};
        for(int vertex=0;vertex<m->mesh_vertnum[mesh];++vertex) {
            // MuJoCo centres and rotates each mesh around its principal axes.
            // Both the asset inverse and the real geom world transform must undo it.
            near(add(rotate(rotation,mesh_vertex(m,mesh,vertex)),translation),obj.vertices[vertex],
                 vertex_tolerance,"Asset recenter/scale "+obj.name);
        }
        for(int face=0;face<m->mesh_facenum[mesh];++face) for(int corner=0;corner<3;++corner) {
            const int address=3*(m->mesh_faceadr[mesh]+face)+corner;
            require(m->mesh_face[address]==obj.triangles[face].vertices[corner],"Source winding/topology changed "+obj.name);
            const int normal_index=m->mesh_normaladr[mesh]+m->mesh_facenormal[address];
            const float* normal=m->mesh_normal+3*normal_index;
            const auto expected=obj.normals[obj.triangles[face].normals[corner]];
            near(rotate(rotation,{normal[0],normal[1],normal[2]}),expected,3e-6,"Corner normal "+obj.name);
        }
        for(int core=0;core<2;++core) {
            const int geom=id(m,mjOBJ_GEOM,"core"+std::to_string(core)+"_"+obj.name);
            require(m->geom_type[geom]==mjGEOM_MESH&&m->geom_dataid[geom]==mesh,"Visual asset replaced "+obj.name);
            require(m->geom_group[geom]==0,"Core visual geometry must be visible in group0");
            require(m->geom_contype[geom]==0&&m->geom_conaffinity[geom]==0,"Detailed visual must not become a convex contact proxy");
            for(int vertex=0;vertex<m->mesh_vertnum[mesh];++vertex)
                near(world_vertex(scene,geom,vertex),add(obj.vertices[vertex],{0,2.0*core,1}),
                     vertex_tolerance,"Actual world scale/orientation/registration "+obj.name);
        }
        total_triangles+=obj.triangles.size();total_vertices+=obj.vertices.size();
    }
    require(objects.size()==60,"Expected all60 evaluated object/material meshes");
    require(total_triangles==34792,"Evaluated Blender geometry must include all34792 triangles");
    std::cout<<"All60 core assets /120 geoms: "<<total_vertices<<" vertices, "<<total_triangles
             <<" triangles, per-corner normals and actual world placement preserved\n";
}

bool marker_mask(double y,double z) {
    const auto within=[](double value,double lo,double hi){return value>lo&&value<hi;};
    if(within(y,10,40)&&within(z,-40,-10))return y>32||z<-32;
    if(within(y,-40,-10)&&within(z,-40,-10))return y<-32||z<-32;
    if(within(y,-40,-10)&&within(z,10,40))return y<-32||z>32;
    return (within(y,10,18)&&within(z,32,40)) ||
           (within(y,26,40)&&within(z,26,40)&&(y>32||z>32)) ||
           (within(y,32,40)&&within(z,10,18));
}
void marker_and_lights(Scene& scene) {
    const auto* m=scene.model.get();const auto* d=scene.data.get();
    for(int core=0;core<2;++core) {
        std::array<int,6> marker{};std::set<int> material_ids;
        for(int i=0;i<6;++i)marker[i]=id(m,mjOBJ_GEOM,"core"+std::to_string(core)+"_pillar_L_light_"+std::to_string(i+1));
        // An offset 1mm grid checks the entire80mm asymmetric mask,
        // including all gaps. This catches a mirrored, filled or symmetric marker.
        // Distinct offsets avoid querying exactly on shared triangulation diagonals.
        for(int iy=-46;iy<46;++iy)for(int iz=-46;iz<46;++iz) {
            const double y=iy+.413,z=iz+.617;
            const mjtNum point[3]={0,y*.001+2*core,z*.001+1},direction[3]={-1,0,0};
            bool hit=false;
            for(const int geom:marker) {
                const double distance=mj_rayMesh(m,d,geom,point,direction);
                if(distance>=0) {hit=true;near(distance,.1,vertex_tolerance,"Actual MARK front plane");}
            }
            require(hit==marker_mask(y,z),"Asymmetric MARK mismatch at mm "+std::to_string(y)+","+std::to_string(z));
        }
        for(int i=1;i<=36;++i) {
            const int geom=id(m,mjOBJ_GEOM,"core"+std::to_string(core)+"_back_light_bar_"+std::to_string(i));
            const int mat=m->geom_matid[geom];require(mat>=0,"Fan light material missing");
            material_ids.insert(mat);
            require(m->mat_emission[mat]>0,"Fan segment lost emissive material");
            const int base=id(m,mjOBJ_BODY,"core"+std::to_string(core)+"_base");
            require(m->geom_bodyid[geom]==base,"Station fan must remain on the stationary shell");
        }
        require(material_ids.size()==36,"All36 fan segments must have independently addressable materials");
    }
    std::cout<<"MARK mask verified at16928 grid points /101568 mesh-ray queries;36 independent fan lights per core\n";
}

Vec moved_point(Vec point,double lift,double flip,double twist) {
    point[0]+=.169;point[2]+=-.208+lift;
    const Vec flipped={std::cos(flip)*point[0]-std::sin(flip)*point[2],point[1],
                       std::sin(flip)*point[0]+std::cos(flip)*point[2]};
    return {-.169+flipped[0],std::cos(twist)*flipped[1]-std::sin(twist)*flipped[2],
            .208+std::sin(twist)*flipped[1]+std::cos(twist)*flipped[2]};
}
void motion_and_fixed_p(Scene& scene,const std::vector<Obj>& objects) {
    const auto* m=scene.model.get();auto* d=scene.data.get();
    // Independent expected transforms from P=(-169,0,208)mm, R_Q(+X) R_P(-Y).
    const std::array<std::array<double,3>,5> poses={{{0,0,0},{.1,0,0},{.1,pi/2,0},
                                                  {.1,pi/2,pi/2},{.05,pi/4,-pi/3}}};
    for(const auto& pose:poses) {
        for(int core=0;core<2;++core) {
            scene.joint(core,"lift",pose[0]);scene.joint(core,"flip",pose[1]);scene.joint(core,"twist",pose[2]);
        }
        mj_forward(m,d);
        for(int core=0;core<2;++core) {
            const Vec offset={0,2.0*core,1};
            const int p=id(m,mjOBJ_JOINT,"core"+std::to_string(core)+"_flip");
            near({d->xanchor[3*p],d->xanchor[3*p+1],d->xanchor[3*p+2]},add({-.169,0,.208},offset),1e-12,
                 "P anchor must not translate with lift or flip");
            const int tip=id(m,mjOBJ_SITE,"core"+std::to_string(core)+"_tip");
            near({d->site_xpos[3*tip],d->site_xpos[3*tip+1],d->site_xpos[3*tip+2]},
                 add(moved_point({0,0,0},pose[0],pose[1],pose[2]),offset),1e-12,"E trajectory about fixed P");
            for(const auto& obj:objects) {
                if(obj.name=="pillar_slide")continue; // Its guide linkage remains an explicit approximation.
                const bool moving=obj.name.rfind("pillar_",0)==0;
                const int geom=id(m,mjOBJ_GEOM,"core"+std::to_string(core)+"_"+obj.name);
                for(std::size_t vertex=0;vertex<obj.vertices.size();++vertex) {
                    const Vec expected=moving?moved_point(obj.vertices[vertex],pose[0],pose[1],pose[2]):obj.vertices[vertex];
                    near(world_vertex(scene,geom,static_cast<int>(vertex)),add(expected,offset),vertex_tolerance,
                         "Rigid-body motion / stationary shell "+obj.name);
                }
            }
        }
    }
    std::cout<<"Five independent lift/P/Q poses preserve every pillar and shell vertex; P remains fixed\n";
}
} // namespace

int main() {
    try {
        std::vector<Obj> objects;
        for(const auto& entry:std::filesystem::directory_iterator(std::filesystem::path(TECH_CORE_SOURCE_DIR)/"models/meshes/core"))
            if(entry.path().extension()==".obj")objects.push_back(read_obj(entry.path()));
        std::sort(objects.begin(),objects.end(),[](const Obj& a,const Obj& b){return a.name<b.name;});
        Scene scene;
        complete_geometry_roundtrip(scene,objects);
        marker_and_lights(scene);
        motion_and_fixed_p(scene,objects);
        std::cout<<"Geometry fidelity contracts passed\n";
        return 0;
    }catch(const std::exception& error) {
        std::cerr<<"Geometry fidelity failure: "<<error.what()<<'\n';return 1;
    }
}
