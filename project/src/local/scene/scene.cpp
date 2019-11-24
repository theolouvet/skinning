

#include <GL/glew.h>

#include "scene.hpp"
#include "../../lib/opengl/glutils.hpp"

#include "../../lib/perlin/perlin.hpp"
#include "../../lib/interface/camera_matrices.hpp"

#include "../interface/myWidgetGL.hpp"

#include <cmath>


#include <string>
#include <sstream>
#include "../../lib/mesh/mesh_io.hpp"

#include "../../skinning/skeleton_parent_id.hpp"
#include "../../skinning/skeleton_geometry.hpp"

#include <chrono>
#include <ctime>



using namespace cpe;

static cpe::mesh build_ground(float const L,float const h)
{
    mesh m;
    m.add_vertex(vec3(-L, h,-L));
    m.add_vertex(vec3(-L, h, L));
    m.add_vertex(vec3( L, h, L));
    m.add_vertex(vec3( L, h,-L));

    m.add_triangle_index({0,2,1});
    m.add_triangle_index({0,3,2});

    m.fill_color(vec3(0.8,0.9,0.8));

    return m;
}

cpe::vertex_weight_parameter attach_weight(int kv, int Nv){
            
    vertex_weight_parameter weight;
    weight[0].joint_id = 0;
    weight[1].joint_id = 1;
    if (kv <= Nv/2){   
        weight[0].weight = 1;
        weight[1].weight = 0;
    }
    else if (kv > Nv/2){
        weight[0].weight = 0;
        weight[1].weight = 1;
    }
    return weight;
}

cpe::vertex_weight_parameter attach_weight(float v){
            
    vertex_weight_parameter weight;
    weight[0].joint_id = 0;
    weight[0].weight = 1.0f - v;
    weight[1].joint_id = 1;
    weight[1].weight = v;
    return weight;
}

static cpe::mesh_skinned build_cylindre(int Nu, int Nv, float radius, float length)
{
    mesh_skinned m;
    for (int ku=0;ku<Nu;++ku)
    {
        float const u = float (ku)/(Nu);
        for(int kv=0 ; kv<Nv ; ++kv)
        {
            float const v = float (kv) /(Nv-1);
            float const x = radius * cos(2.0f*M_PI*u);
            float const y = radius * sin(2.0f*M_PI*u);
            float const z = length *v   ;

            m.add_vertex({x,y,z});
        
            //ajout des poids
            //m.add_vertex_weight(attach_weight(kv, Nv));
            m.add_vertex_weight(attach_weight(v));
        }
    }
    for(int ku=0;ku<Nu;++ku)
    {
        for(int kv=0 ; kv<Nv-1 ; ++kv)
        {
            int const k0=Nv*ku+kv;
            int const k1=Nv*((ku+1)%Nu)+kv;
            int const k2=Nv*((ku+1)%Nu)+(kv+1);
            int const k3=Nv*ku+(kv+1);

            m.add_triangle_index({k0,k1,k2});
            m.add_triangle_index({k0,k2,k3});
        }
    }
    m.fill_color(vec3(1.0,0,0));

    return m;
}



void scene::build_skeletton(){
     sk_cylinder_parent_id.push_back(-1);
    sk_cylinder_parent_id.push_back(0);
    sk_cylinder_parent_id.push_back(1);

    cpe::skeleton_joint j1 = cpe::skeleton_joint(vec3(0,0,0),quaternion(0,0,0,1));
    cpe::skeleton_joint j2 = cpe::skeleton_joint(vec3(0,0,25),quaternion(0,0,0,1));
    cpe::skeleton_joint j3 = cpe::skeleton_joint(vec3(0,0,25),quaternion(0,0,0,1));

    sk_cylinder_bind_pose.clear();

    sk_cylinder_bind_pose.push_back(j1);
    sk_cylinder_bind_pose.push_back(j2);
    sk_cylinder_bind_pose.push_back(j3);

}

void scene::build_skeletton_animation(){
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);
    
    sk_cylinder_bind_pose[1].orientation.set_axis_angle(vec3(0,1,0), -30*M_PI/180);
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);
    sk_cylinder_bind_pose[1].orientation.set_axis_angle(vec3(0,1,0), -60*M_PI/180);
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);
    sk_cylinder_bind_pose[1].orientation.set_axis_angle(vec3(0,1,0), -90*M_PI/180);
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);

    sk_cylinder_bind_pose[1].orientation = quaternion(0, 0, 0, 1);

    i=0;
}




void scene::load_scene()
{

    start = std::chrono::system_clock::now();
    //*****************************************//
    // Preload default structure               //
    //*****************************************//
    texture_default = load_texture_file("data/white.jpg");
    shader_mesh     = read_shader("shaders/shader_mesh.vert",
                                  "shaders/shader_mesh.frag");           PRINT_OPENGL_ERROR();
    shader_skeleton = read_shader("shaders/shader_skeleton.vert",
                                  "shaders/shader_skeleton.frag");       PRINT_OPENGL_ERROR();


    mesh_cylinder = build_cylindre(40,40,4.0f,50.0f);
    mesh_cylinder.fill_empty_field_by_default();
    mesh_cylinder_opengl.fill_vbo(mesh_cylinder);

    //*****************************************//
    // Build ground
    //*****************************************//
    mesh_ground = build_ground(100.0f , -25.0f);
    mesh_ground.fill_empty_field_by_default();
    mesh_ground_opengl.fill_vbo(mesh_ground);


    build_skeletton();
    build_skeletton_animation();


}



void scene::draw_scene()
{
    end = std::chrono::system_clock::now();
    float deltaT = std::chrono::duration_cast<std::chrono::milliseconds>
                                 (end-start).count();
    float alpha =  deltaT/2000.;
    if (deltaT > 2000){
        i++;
        alpha = 0;
        start = std::chrono::system_clock::now();
    }
    if (i > 3){
        i=0;
    }

    //skeleton_geometry const sk_cylinder_global = local_to_global(sk_cylinder_animation[i],sk_cylinder_parent_id);
    skeleton_geometry const sk_cylinder_global = local_to_global(sk_cylinder_animation(i,alpha),sk_cylinder_parent_id);
    std::vector<vec3> const sk_cylinder_bones = extract_bones(sk_cylinder_global,sk_cylinder_parent_id);
    

    skeleton_geometry const sk_cylinder_inverse_bind_pose = inversed(sk_cylinder_bind_pose);
    skeleton_geometry const sk_cylinder_binded = multiply(sk_cylinder_global,sk_cylinder_inverse_bind_pose);
    mesh_cylinder.apply_skinning(sk_cylinder_binded);
    mesh_cylinder.fill_normal();
    mesh_cylinder_opengl.update_vbo_vertex(mesh_cylinder);
    mesh_cylinder_opengl.update_vbo_normal(mesh_cylinder);

    setup_shader_mesh(shader_mesh);
    mesh_cylinder_opengl.draw();

    setup_shader_mesh(shader_mesh);
    mesh_ground_opengl.draw();

      setup_shader_skeleton(shader_skeleton);
    draw_skeleton(sk_cylinder_bones);

  
}


void scene::setup_shader_mesh(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"normal_matrix"),1,false,cam.normal.pointer());           PRINT_OPENGL_ERROR();

    //load white texture
    glBindTexture(GL_TEXTURE_2D,texture_default);                                                      PRINT_OPENGL_ERROR();
    glLineWidth(1.0f);                                                                                 PRINT_OPENGL_ERROR();

}

void scene::setup_shader_skeleton(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniform3f(get_uni_loc(shader_id,"color") , 0.0f,0.0f,0.0f);                                      PRINT_OPENGL_ERROR();

    //size of the lines
    glLineWidth(3.0f);                                                                                 PRINT_OPENGL_ERROR();
}

void scene::draw_skeleton(std::vector<vec3> const& positions) const
{
    // Create temporary a VBO to store data
    GLuint vbo_skeleton=0;
    glGenBuffers(1,&vbo_skeleton);                                                                     PRINT_OPENGL_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER,vbo_skeleton);                                                        PRINT_OPENGL_ERROR();

    // Update data on the GPU
    glBufferData(GL_ARRAY_BUFFER , sizeof(vec3)*positions.size() , &positions[0] , GL_STATIC_DRAW);    PRINT_OPENGL_ERROR();

    // Draw data
    glEnableClientState(GL_VERTEX_ARRAY);                                                              PRINT_OPENGL_ERROR();
    glVertexPointer(3, GL_FLOAT, 0, 0);                                                                PRINT_OPENGL_ERROR();
    glDrawArrays(GL_LINES,0,positions.size());                                                         PRINT_OPENGL_ERROR();

    // Delete temporary VBO
    glDeleteBuffers(1,&vbo_skeleton);                                                                  PRINT_OPENGL_ERROR();
}

scene::scene()
    :shader_mesh(0)
{}


GLuint scene::load_texture_file(std::string const& filename)
{
    return pwidget->load_texture_file(filename);
}

void scene::set_widget(myWidgetGL* widget_param)
{
    pwidget=widget_param;
}



