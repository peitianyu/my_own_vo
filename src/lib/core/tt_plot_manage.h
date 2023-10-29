#ifndef __PLOT_MANAGE_H__
#define __PLOT_MANAGE_H__

#include <memory>
#include "tt_plot.h"

class PlotManager {
public:
    PlotManager(size_t width, size_t height) {
        plot_ = std::make_unique<signalsmith::plot::Plot2D>(width, height);
    }

    signalsmith::plot::Line2D& line(uint mode = 0) {
        return plot_->line(mode);
    }

    signalsmith::plot::Line2D& draw_line(double x, double y, double x_end, double y_end, uint mode) {
        auto& line = plot_->line(mode);
        line.add(x, y);
        line.add(x_end, y_end);

        return line;
    }

    signalsmith::plot::Line2D& draw_rect(signalsmith::plot::Line2D& line, double x, double y, double width, double height) {
        line.add(x, y);
        line.add(x + width, y);
        line.add(x + width, y + height);
        line.add(x, y + height);
        line.add(x, y);

        return line;
    }

    signalsmith::plot::Line2D& draw_rect(double x, double y, double width, double height, uint mode) {
        auto& line = plot_->line(mode);
        line.add(x, y);
        line.add(x + width, y);
        line.add(x + width, y + height);
        line.add(x, y + height);
        line.add(x, y);

        return line;
    }

    signalsmith::plot::Line2D& draw_circle(double x, double y, double radius, uint mode) {
        auto& line = plot_->line(mode);
        constexpr size_t point_num = 100;
        for (size_t i = 0; i < point_num; ++i) {
            double angle = 2 * M_PI * i / point_num;
            line.add(x + radius * cos(angle), y + radius * sin(angle));
        }
        line.add(x + radius, y);

        return line;
    }

    void draw_grid(uint scale, uint mode) {
        double width = plot_->x.drawMax() - plot_->x.drawMin();
        double height = plot_->y.drawMax() - plot_->y.drawMin();

        for (double i = 0; i < width; i += scale) 
            draw_line(i, 0, i, height, mode);
        for (double i = 0; i < height; i += scale) 
            draw_line(0, i, width, i, mode);
    }

    signalsmith::plot::Line2D& draw_marker(signalsmith::plot::Line2D& line, const double& x, const double& y) {
        line.marker(x, y);
        return line;
    }

    signalsmith::plot::Line2D& draw_marker(signalsmith::plot::Line2D& line, const std::vector<double>& xs, const std::vector<double>& ys) {
        for(size_t i = 0; i < xs.size(); ++i) {
            line.marker(xs[i], ys[i]);
        }
        return line;
    }
    
    void show_legend(signalsmith::plot::Line2D& line, const std::string& str, double x, double y) {
        plot_->legend(x, y).add(line, str);
    }
    void write(const std::string& file_name) {
        plot_->write(file_name);
    }
private:
    std::unique_ptr<signalsmith::plot::Plot2D> plot_;
};



#endif // __PLOT_MANAGE_H__