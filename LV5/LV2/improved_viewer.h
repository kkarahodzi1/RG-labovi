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

#ifndef RG_LV_IMPROVED_VIEWER_H
#define RG_LV_IMPROVED_VIEWER_H

#include <easy3d/viewer/viewer.h>
#include <easy3d/viewer/drawable_triangles.h>
#include <easy3d/viewer/drawable_lines.h>
#include <easy3d/viewer/drawable_points.h>


class ImprovedViewer : public easy3d::Viewer
{
public:
    ImprovedViewer(const std::string& title = "s");

protected:
    bool mouse_press_event(int x, int y, int button, int modifiers) override;
    bool key_press_event(int key, int modifiers) override;
    bool mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers) override;
    std::string usage() const override;
    easy3d::TrianglesDrawable* MakeSimplePolygon(std::vector<easy3d::vec3> points);
    easy3d::LinesDrawable* MakeSimplePolygonOutline(std::vector<easy3d::vec3> points);

private:
    void update_rendering();
    std::vector<easy3d::vec3> click_points;
    std::vector<easy3d::vec3> poly_points;
    easy3d::Drawable *last_one;

    easy3d::PointsDrawable* randPoints;
    easy3d::LinesDrawable* randRectangle;

    easy3d::LinesDrawable *BruteForceConvexHullN4(std::vector<easy3d::vec3> points);

    easy3d::LinesDrawable *BruteForceConvexHullN2(std::vector<easy3d::vec3> points);
};


#endif // RG_LV_IMPROVED_VIEWER_H
