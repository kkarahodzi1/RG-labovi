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
    srand (time(NULL));
    last_one = nullptr;
    randPoints = nullptr;
    randRectangle = nullptr;
    tacke=nullptr;
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

        click_points.emplace_back(x,y,0);

        auto* drawable = new PointsDrawable("vertices");
        drawable->update_vertex_buffer(std::vector<vec3>{ camera_->unprojectedCoordinatesOf(vec3(x,y,0))});
        drawable->set_default_color(vec3(0,0,0));
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
    return false;
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
    return {p2.y-p1.y, p1.x-p2.x, p1.y*p2.x -  p1.x*p2.y};
}
bool ArePointsOnSameSideOfLine(const std::vector<easy3d::vec3>& points, vec3 l)
{
    double s=0;
    for(auto & point : points)
    {
        double v =  l.x*point.x + l.y*point.y +l.z;
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
        double a = r.x - q.x, b = r.y - q.y, c = s.x - q.x, d = s.y - q.y;
        double e = a*d - b*c;
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

    auto* polygon = new TrianglesDrawable("polygon");
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
        double a = r.x - q.x, b = r.y - q.y, c = s.x - q.x, d = s.y - q.y;
        double e = a*d - b*c;
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

    auto* polygon = new LinesDrawable("polygon");
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

vec3 LineSegmentsIntersection(vec3 s1R, vec3 s1Q, vec3 s2R, vec3 s2Q )
{
    double a = s1R.x - s1Q.x,
           b = s2Q.x - s2R.x,
           c = s2Q.x - s1Q.x,
           d = s1R.y - s1Q.y,
           e = s2Q.y - s2R.y,
           f = s2Q.y - s1Q.y;

    double delta = a*e - b*d,
           delta1 = c*e - b*f;
    if(delta)
    {
        double delta2 = a*f - c*d;
        if((delta1>=0  && delta2>=0 && delta1<=delta && delta2<=delta) || (delta1<=0  && delta2<=0 && delta1>=delta && delta2>=delta))
        {
            double lambda = delta1/delta;
            return vec3((1-lambda)*s1Q.x+lambda*s1R.x, (1-lambda)*s1Q.y+lambda*s1R.y,0);
        }
    }
    else if (!delta1)
    {
        if((c<=0 || c<=b) && (c>=0 || c>=b) && (f<=0 || f<=e) && (f>=0 || f>=e))
            return s1Q;
        if((c>0 || c>a) && (c<0 || c>a) && (f>0 || f>d) && (f<0 || f>d))
            return s2Q;
        if((a>=c || s1R.x>=s2R.x) && (a<=c || s1R.x<=s2R.x) && (d>=f || s1R.y>=s2R.y) && (d<=f || s1R.y<=s2R.y))
            return s1R;
        if((b<c || s2R.x>s1R.x) && (b>c || s2R.x<s1R.x) && (e<f || s2R.y>s1R.y) && (e>f || s2R.y<s1R.y))
            return s2R;
    }
    return vec3(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0);
}

bool ImprovedViewer::key_press_event(int key, int modifiers) {
    if (key == GLFW_KEY_N) {

        int x,y;
        std::cout << "Enter coordinates of a point(x,y): ";
        std::cin >> x >> y;

        auto* drawable = new PointsDrawable("vertices");
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
            point_vec.emplace_back(x,y,0);
        }

        point_file.close();

        auto* drawable = new PointsDrawable("vertices");
        drawable->update_vertex_buffer(point_vec);
        drawable->set_default_color(vec3(1,0,1));
        drawable->set_point_size(5);
        drawable->set_impostor_type(PointsDrawable::SPHERE);
        this->add_drawable(drawable);

        point_file.open("indices.txt", std::ifstream::in);

        std::vector<unsigned int> indices;
        while (point_file.good()) {
            int x, y;
            point_file >> x >> y;
            indices.push_back(x);
            indices.push_back(y);
        }
        point_file.close();
        auto *index_drawable = new LinesDrawable("vertices");
        index_drawable->update_vertex_buffer(point_vec);
        index_drawable->update_index_buffer(indices);
        index_drawable->set_default_color(vec3(1, 1, 0));
        this->add_drawable(index_drawable);

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

        std::vector<unsigned int> indices;
        std::vector<easy3d::vec3> pom;
        std::vector<easy3d::vec3> colours;
        int broj, i = 0;
        point_file >> broj;
        while (point_file.good()) {
            int x, y;
            point_file >> x >> y;
            poly_points.emplace_back(x, y, 0);
            pom.push_back(camera_->unprojectedCoordinatesOf(vec3(x, y, 0)));
            colours.emplace_back(1, 1, 0);
            indices.push_back(i);
            indices.push_back((i + 1) % broj);
            i++;
        }
        point_file.close();
        if (last_one) {
            this->delete_drawable(last_one);
            last_one = nullptr;
        }
        last_one = MakeSimplePolygonOutline(poly_points);
        last_one->set_default_color(vec3(1, 1, 0));
        this->add_drawable(last_one);
        camera()->showEntireScene();
        update();

    }
    else if (key == GLFW_KEY_C)
    {
        if (!click_points.empty()) {
            if (last_one != nullptr) {
                bool f = false;
                vec3 p = click_points[click_points.size() - 1];
                for (int i = 0; i < poly_points.size(); i++) {
                    int j = (i + 1) % poly_points.size();
                    if ((poly_points[j].y <= p.y && p.y <= poly_points[i].y &&
                         AreaOfTriangleAux(poly_points[j], poly_points[i], p) > 0) ||
                        (poly_points[i].y <= p.y && p.y <= poly_points[j].y &&
                         AreaOfTriangleAux(poly_points[i], poly_points[j], p) > 0))
                        f = !f;
                }
                std::cout << (f ? "UNUTRA\n" : "VANI\n");
            }
        }
    }
    else if (key == GLFW_KEY_H)
    {
        if(last_one)
        {
            this->delete_drawable(last_one);
            last_one = nullptr;
        }

        std::cout << "Unesi: \n\t1 - za brute-force algoritam kompleksnosti n^4\n\t2 - za brute-force algoritam kompleksnosti n^2\n";
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
    else if (key == GLFW_KEY_R)
    {
        if(randPoints)
        {
            this->delete_drawable(randPoints);
            randPoints = nullptr;
        }
        if(randRectangle)
        {
            this->delete_drawable(randRectangle);
            randRectangle = nullptr;
        }

        randPoints = new PointsDrawable("tacke");
        randRectangle = new LinesDrawable("pravougaonik");

        std::vector<vec3> tacke(100), tacke_skalirano(100);
        for(int i = 0; i<100; i++)
        {
            tacke[i] = vec3(rand()%camera_->screenWidth(), rand()%camera_->screenHeight(),0);
            tacke_skalirano[i] = camera_->unprojectedCoordinatesOf(tacke[i]);
        }

        int left = rand()%(camera_->screenWidth()/2), right = rand()%(camera_->screenWidth()/2)+camera_->screenWidth()/2,
            top=rand()%(camera_->screenWidth()/2), bottom=rand()%(camera_->screenWidth()/2)+camera_->screenWidth()/2;

        if(right<left)
            std::swap(left,right);
        if(bottom<top)
            std::swap(top,bottom);

        std::vector<vec3> pravougaonik = {camera_->unprojectedCoordinatesOf(vec3(left,top,0)),
                                          camera_->unprojectedCoordinatesOf(vec3(right,top,0)),
                                          camera_->unprojectedCoordinatesOf(vec3(right,bottom,0)),
                                          camera_->unprojectedCoordinatesOf(vec3(left,bottom,0))};
        randRectangle->update_vertex_buffer(pravougaonik);
        randRectangle->update_index_buffer({0,1,1,2,2,3,3,0});
        randRectangle->set_default_color(vec3(0.5,0.5,0.5));
        this->add_drawable(randRectangle);

        std::vector<vec3> boje;
        int broj = 0;
        for(int i=0; i<100; i++)
            if(tacke[i].x>=left && tacke[i].x<=right && tacke[i].y>=top && tacke[i].y<=bottom)
                boje.push_back(vec3(0,1,0)), broj++;
            else
                boje.push_back(vec3(1,0,0));

        std::cout << "Unutar pravougaonika se nalaze "<< broj << " tacke, i to:\n";
        for(int i=0; i<100; i++)
            if(tacke[i].x>=left && tacke[i].x<=right && tacke[i].y>=top && tacke[i].y<=bottom)
                std::cout << "( " << tacke[i].x << ", " << tacke[i].y  << ")\n";

        randPoints->update_vertex_buffer(tacke_skalirano);
        randPoints->set_default_color(vec3(1,0,0));
        randPoints->update_color_buffer(boje);
        randPoints->set_point_size(5);
        randPoints->set_impostor_type(PointsDrawable::SPHERE);
        this->add_drawable(randPoints);
        update();
    }
    else if (key == GLFW_KEY_D)
    {
        if(last_one)
        {
            this->delete_drawable(last_one);
            last_one = nullptr;
        }

        if(tacke)
        {
            this->delete_drawable(tacke);
            tacke = nullptr;
        }
        srand(time(NULL));
        int n = rand()%10+10, m=rand()%10+10;
        std::vector<unsigned int> indices;
        std::vector<vec3> verteces, intersections;

        for(int i=0; i<n; i++)                      
        {
            double x1 = rand()%1500-750,x2 = rand()%1500-750, y = rand()%1500-750;
            if(x1>x2)
                std::swap(x1,x2);


            verteces.push_back(vec3(x1, y, 0));
            indices.push_back(verteces.size()-1);
            verteces.push_back(vec3(x2, y, 0));
            indices.push_back(verteces.size()-1);
        }

        for(int i=0; i<m; i++)
        {
            double x = rand()%1500-750, y1 = rand()%1500-750, y2 = rand()%1500-750;
            if(y1>y2)
                std::swap(y1,y2);

            verteces.push_back(vec3(x, y1, 0));
            indices.push_back(verteces.size()-1);
            verteces.push_back(vec3(x, y2, 0));
            indices.push_back(verteces.size()-1);

            for(int j=0; j<n; j++)
            {
                vec3 tacka = LineSegmentsIntersection(vec3(x, y1, 0), vec3(x, y2, 0), verteces[2*j], verteces[2*j+1]);
                if(tacka.x < 1500 && tacka.x>-1500)
                    intersections.push_back(tacka);
            }
        }
        last_one = new LinesDrawable("duži");
        last_one->update_index_buffer(indices);
        last_one->update_vertex_buffer(verteces);



        tacke = new PointsDrawable("presjeci");
        tacke->update_vertex_buffer(intersections);
        tacke->set_default_color(vec3(1,0,1l));
        tacke->set_point_size(5);
        tacke->set_impostor_type(PointsDrawable::SPHERE);

        this->add_drawable(last_one);
        this->add_drawable(tacke);
        camera()->showEntireScene();
        update();
    }
    else if(key == GLFW_KEY_L)
    {

        if(last_one)
        {
            this->delete_drawable(last_one);
            last_one = nullptr;
        }
        if(tacke)
        {
            this->delete_drawable(tacke);
            tacke = nullptr;
        }

        srand(time(NULL));
        int n = rand()%10+20;
        std::vector<unsigned int> indices;
        std::vector<vec3> verteces, intersections;

        for(int i=0; i<n; i++)
        {
            double x1 = rand()%1500-750,x2 = rand()%1500-750, y1 = rand()%1500-750, y2 = rand()%1500-750;
            if(y1>y2)
                std::swap(y1,y2);

            verteces.push_back(vec3(x1, y1, 0));
            indices.push_back(verteces.size()-1);
            verteces.push_back(vec3(x2, y2, 0));
            indices.push_back(verteces.size()-1);

            for(int j=0; j<i; j++)
            {
                vec3 tacka = LineSegmentsIntersection(verteces[2*i], verteces[2*i+1], verteces[2*j], verteces[2*j+1]);
                if(isfinite(tacka.x))
                    intersections.push_back(tacka);
            }
        }
        last_one = new LinesDrawable("duži");
        last_one->update_index_buffer(indices);
        last_one->update_vertex_buffer(verteces);



        tacke = new PointsDrawable("presjeci");
        tacke->update_vertex_buffer(intersections);
        tacke->set_default_color(vec3(1,1,0));
        tacke->set_point_size(5);
        tacke->set_impostor_type(PointsDrawable::SPHERE);

        this->add_drawable(last_one);
        this->add_drawable(tacke);
        camera()->showEntireScene();
        update();

    }
    else
        return Viewer::key_press_event(key, modifiers);
    return false;
}



