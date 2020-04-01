/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "improved_viewer.h"

#include <fstream>
#include <algorithm>

#include <easy3d/core/point_cloud.h>
#include <easy3d/viewer/camera.h>
#include <easy3d/viewer/drawable_points.h>
#include <easy3d/viewer/drawable_triangles.h>
#include <easy3d/viewer/drawable_lines.h>
#include <easy3d/algo/point_cloud_normals.h>
#include <3rd_party/glfw/include/GLFW/glfw3.h>	// for the KEYs

using namespace easy3d;

ImprovedViewer::ImprovedViewer(const std::string& title) : Viewer(title) {
    last_one = nullptr;
    camera()->setType(Camera::ORTHOGRAPHIC);
    camera()->setUpVector(vec3(0, -1, 0));
    camera()->setViewDirection(vec3(0,0,1));
    camera()->setSceneRadius(camera()->screenHeight());
    camera()->setPosition(vec3(0, 0, 0));
}


std::string ImprovedViewer::usage() const {
    return ("----------- ImprovedViewer usage------------ \n"
            "Right click generates vertex\n"
            "Key press of n enables input\n"
            "Key press of f enables read\n"
            "------------------------------------------------ \n");
}

bool ImprovedViewer::mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers) {
    return false;
}

bool ImprovedViewer::mouse_press_event(int x, int y, int button, int modifiers)  {
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {

        click_points.push_back(vec3(x,y,0));

        PointsDrawable* drawable = new PointsDrawable("vertices");
        drawable->update_vertex_buffer(std::vector<vec3>{ camera_->unprojectedCoordinatesOf(vec3(x,y,0))});
        drawable->set_default_color(vec3(1,0,0));
        drawable->set_point_size(5);
        drawable->set_impostor_type(PointsDrawable::SPHERE);
        this->add_drawable(drawable);
        camera()->setUpVector(vec3(0, -1, 0));
        camera()->setViewDirection(vec3(0,0,1));
        camera()->setSceneRadius(camera()->screenHeight());
        camera()->setPosition(vec3(0, 0, 0));
        camera()->showEntireScene();
        camera()->centerScene();
        update();

        }
    else if (button == GLFW_MOUSE_BUTTON_LEFT)
    {

    }
    else
        return Viewer::mouse_press_event(x,y,button, modifiers);
}

double AreaOfTriangleAux(vec3 p1, vec3 p2, vec3 p3)
{
    return (p2.x-p1.x)*(p3.y-p1.y)-(p3.x-p1.x)*(p2.y-p1.y);
}

bool isPointInsideTriangle(easy3d::vec3 point, easy3d::vec3 q, easy3d::vec3 r, easy3d::vec3 s)
{
    double a = AreaOfTriangleAux(q,r,s);
    return a*AreaOfTriangleAux(point,q,r) > 0 && a*AreaOfTriangleAux(point,r,s) > 0 && a*AreaOfTriangleAux(point,s,q) > 0;
}

vec3 LineThroughTwoPoints(vec3 p1, vec3 p2)
{
    return vec3(p2.y-p1.y, p1.x-p2.x, p1.y*p2.x -  p1.x*p2.y);
}
bool ArePointsOnSameSideOfLine(std::vector<easy3d::vec3> points, vec3 l)
{
    double s=0;
    for(int i=0; i<points.size(); i++)
    {
        double v =  l.x*points[i].x + l.y*points[i].y +l.z;
        if(v!=0)
        {
            if(s==0)
            {
                s = v;
            }
            else if(s*v<0)
                return false;
        }
    }
    return true;
}

easy3d::TrianglesDrawable *ImprovedViewer::MakeSimplePolygon(std::vector<easy3d::vec3> points) {
    int b=0;
    for(int i=1; i<points.size();i++)
        if(points[i].x<points[b].x || (points[i].x==points[b].x && points[i].y<points[b].y))
            b=i;

    auto q = points[b];
    auto comp = [q](vec3 r, vec3 s)
    {
        int a = r.x - q.x, b = r.y - q.y, c = s.x - q.x, d = s.y - q.y;
        int e = a*d - b*c;
        if(e!=0)
            return e>0;
        return a*a+b*b < c*c+d*d;
    };

    std::sort(points.begin(), points.end(), comp);

    std::vector<unsigned int> indices;


    for(int i=1; i<points.size()-1; i++)
    {
        indices.push_back(0);
        indices.push_back(i);
        indices.push_back(i+1);
    }


    std::vector<vec3> unprojected(points.size());

    for(int i=0; i<points.size(); i++)
    {
        unprojected[i] = camera_->unprojectedCoordinatesOf(points[i]);
    }

    easy3d::TrianglesDrawable* polygon = new TrianglesDrawable("polygon");
    polygon->update_vertex_buffer(unprojected);
    polygon->update_index_buffer(indices);
    return polygon;
}

easy3d::LinesDrawable *ImprovedViewer::MakeSimplePolygonOutline(std::vector<easy3d::vec3> points) {
    int b=0;
    for(int i=1; i<points.size();i++)
        if(points[i].x<points[b].x || (points[i].x==points[b].x && points[i].y<points[b].y))
            b=i;

    auto q = points[b];
    auto comp = [q](vec3 r, vec3 s)
    {
        int a = r.x - q.x, b = r.y - q.y, c = s.x - q.x, d = s.y - q.y;
        int e = a*d - b*c;
        if(e!=0)
            return e>0;
        return a*a+b*b < c*c+d*d;
    };

    std::sort(points.begin(), points.end(), comp);

    std::vector<unsigned int> indices;


    for(int i=0; i<points.size(); i++)
    {
        indices.push_back(i);
        indices.push_back((i+1)%points.size());
    }


    std::vector<vec3> unprojected(points.size());

    for(int i=0; i<points.size(); i++)
    {
        unprojected[i] = camera_->unprojectedCoordinatesOf(points[i]);
    }

    easy3d::LinesDrawable* polygon = new LinesDrawable("polygon");
    polygon->update_vertex_buffer(unprojected);
    polygon->update_index_buffer(indices);
    return polygon;
}

easy3d::LinesDrawable *ImprovedViewer::BruteForceConvexHullN4(std::vector<easy3d::vec3> points)
{
    std::vector<easy3d::vec3> hull;
    for(int p=0; p<points.size(); p++)
    {
        bool t = true;
        for(int i=0; i<points.size() && t; i++)
        {
            if(i != p)
            {
                for(int j=i+1; j<points.size() && t; j++)
                {
                    if(j != p)
                    {
                        for(int k=j+1; k<points.size() && t; k++)
                        {
                            if(k!=p && isPointInsideTriangle(points[p],points[i],points[j],points[k]))
                                t = false;
                        }
                    }
                }
            }
        }
        if(t)
            hull.push_back(points[p]);
    }
    return MakeSimplePolygonOutline(hull);
}

easy3d::LinesDrawable *ImprovedViewer::BruteForceConvexHullN2(std::vector<easy3d::vec3> points)
{
    std::vector<easy3d::vec3> hull;
    std::vector<bool> b(points.size(),false);

    for(int i=0; i<points.size(); i++)
    {
        for(int j=i+1; j<points.size(); j++)
        {
            if(ArePointsOnSameSideOfLine(points, LineThroughTwoPoints(points[i],points[j])))
            {
                b[i] = b[j] = true;
            }
        }
    }

    for(int i=0; i<points.size(); i++)
    {
        if(b[i])
        {
            hull.push_back(points[i]);
        }
    }

    return MakeSimplePolygonOutline(hull);
}

bool ImprovedViewer::key_press_event(int key, int modifiers) {
    if (key == GLFW_KEY_N) {

        int x,y;
        std::cout << "Enter coordinates of a point(x,y): ";
        std::cin >> x >> y;

        PointsDrawable* drawable = new PointsDrawable("vertices");
        drawable->update_vertex_buffer(std::vector<vec3>{ camera_->unprojectedCoordinatesOf(vec3(x,y,0))});
        drawable->set_default_color(vec3(0,0,1));
        drawable->set_point_size(5);
        drawable->set_impostor_type(PointsDrawable::SPHERE);
        this->add_drawable(drawable);

        camera()->showEntireScene();
        update();
    }
    else if (key == GLFW_KEY_K) {

        std::ifstream point_file;

        point_file.open("points.txt", std::ifstream::in);

        std::vector<vec3> point_vec;

        while(point_file.good())
        {
            int x,y;
            point_file >> x >> y;
            point_vec.push_back(vec3(x,y,0));
        }

        point_file.close();

        PointsDrawable* drawable = new PointsDrawable("vertices");
        drawable->update_vertex_buffer(point_vec);
        drawable->set_default_color(vec3(1,0,1));
        drawable->set_point_size(5);
        drawable->set_impostor_type(PointsDrawable::SPHERE);
        this->add_drawable(drawable);

        point_file.open("indices.txt", std::ifstream::in);

        if(point_file.badbit)
        {
            std::vector<unsigned int> indices;
            while(point_file.good())
            {
                int x,y;
                point_file >> x >> y;
                indices.push_back(x);
                indices.push_back(y);
            }

            point_file.close();

            LinesDrawable* index_drawable = new LinesDrawable("vertices");
            index_drawable->update_vertex_buffer(point_vec);
            index_drawable->update_index_buffer(indices);
            index_drawable->set_default_color(vec3(1,1,0));
            this->add_drawable(index_drawable);

        }

        camera()->showEntireScene();
        update();
    }
    else if (key == GLFW_KEY_P)
    {
        if(last_one)
        {
            this->delete_drawable(last_one);
            last_one = nullptr;
        }

        last_one = MakeSimplePolygon(click_points);
        last_one->set_default_color(vec3(1,0,0));

        this->add_drawable(last_one);
        camera()->showEntireScene();
        update();

    }
    else if (key == GLFW_KEY_I)
    {
        std::ifstream point_file;
        point_file.open("polygon.txt", std::ifstream::in);

        if(point_file.badbit)
        {

            std::vector<unsigned int> indices;
            std::vector<easy3d::vec3> pom;
            std::vector<easy3d::vec3> colours;

            int broj,i=0;
            point_file >> broj;


            while(point_file.good())
            {
                int x,y;
                point_file >> x >> y;
                poly_points.push_back(vec3(x,y,0));
                pom.push_back(camera_->unprojectedCoordinatesOf(vec3(x,y,0)));
                colours.push_back(vec3(1,1,0));
                indices.push_back(i);
                indices.push_back((i+1)%broj);
                i++;
            }

            point_file.close();

            if(last_one)
            {
                this->delete_drawable(last_one);
                last_one = nullptr;
            }

            last_one = MakeSimplePolygonOutline(poly_points);
            last_one->set_default_color(vec3(1,1,0));

            this->add_drawable(last_one);
            camera()->showEntireScene();
            update();

        }

    }
    else if (key == GLFW_KEY_C)
    {
        if(last_one && click_points.size())
        {
            bool f = false;
            vec3 p = click_points[click_points.size()-1];
            for(int i = 0; i<poly_points.size(); i++)
            {
                int j = (i+1)%poly_points.size();
                if((poly_points[j].y <= p.y && p.y<=poly_points[i].y && AreaOfTriangleAux(poly_points[j], poly_points[i], p)>0) ||
                   (poly_points[i].y <= p.y && p.y<=poly_points[j].y && AreaOfTriangleAux(poly_points[i], poly_points[j], p)>0))
                 f = (f?false:true);
            }
            std::cout << (f?"UNUTRA\n":"VANI\n");
        }
    }
    else if (key == GLFW_KEY_H)
    {
        if(last_one)
        {
            this->delete_drawable(last_one);
            last_one = nullptr;
        }

        std::cout << "Unesi: \n\t1 - za brute-force algoritam kompleksnosti n^4\n\t2 - za brute-force algoritam kompleksnosti n^2";
        short izbor;
        std::cin >> izbor;
        if(izbor == 1)
        {
            last_one = BruteForceConvexHullN4(click_points);
            last_one->set_default_color(vec3(0,1,0));

            this->add_drawable(last_one);
            camera()->showEntireScene();
            update();
        }
        else if(izbor == 2)
        {
            last_one = BruteForceConvexHullN2(click_points);
            last_one->set_default_color(vec3(0,0,1));

            this->add_drawable(last_one);
            camera()->showEntireScene();
            update();
        }
        else
        {
            std::cout <<"Greska\n";
        }

    }
    else
        return Viewer::key_press_event(key, modifiers);
}



