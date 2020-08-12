/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   07.08.2017
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
* @Project: ImmersiveTripReports 2017-2018
* AdobePatentID="P7840-US"
*/
/*
# Copyright 2020 CPhoto@FIT, Brno University of Technology,
# Faculty of Information Technology,
# Božetěchova 2, 612 00, Brno, Czech Republic
#
# Redistribution and use in source code form, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
#
# 2. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# 3. Redistributions must be pursued only for non-commercial research
#    collaboration and demonstration purposes.
#
# 4. Where separate files retain their original licence terms
#    (e.g. MPL 2.0, Apache licence), these licence terms are announced, prevail
#    these terms and must be complied.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.
*/

#ifndef UTIL_H
#define UTIL_H

//STL headers
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <random>

//BOOST headers
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <osg/Vec3d>
#include <osg/Quat>

#include <Eigen/Dense>

#include <osgDB/XmlParser>


using namespace std;
namespace fs = boost::filesystem;

namespace itr{

class Util{

public:
    /**
     * @brief findFileRecursively
     * Recursively searches for files in a directory. If file_name is not
     * specified, all files with given extension are returned. If no
     * extension is specified, files with the same file_name and various
     * extensions are returned. If none of the file_name or file_ext are
     * specified, all files are returned.
     * @param path path where to start to search
     * @param file_name name of the file (or empty string for all files)
     * without an extension
     * @param file_ext extension of the file (or empty string for the),
     * including extension separator - usually the dot (eg., .txt).
     */
    static vector<string> findFileRecursively(const string &path,
                                      const string &file_name = "",
                                      const string &file_ext = "")
    {
        vector<string> m_file_list;
        if (!path.empty())
        {
            namespace fs = boost::filesystem;

            fs::path apk_path(path);
            fs::recursive_directory_iterator end;

            for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
            {
                const fs::path cp = (*i);
                string ext = fs::extension(cp);
                string basename = fs::basename(cp);
                boost::algorithm::to_lower(ext);
                boost::algorithm::to_lower(basename);
                if (file_name.empty() && !ext.empty())
                {
                    if (ext == file_ext)
                    {
                        m_file_list.push_back(cp.string());
                    }
                }
                else if (!file_name.empty() && ext.empty())
                {
                    if (basename == file_name)
                    {
                        m_file_list.push_back(cp.string());
                    }
                }
                else if (!file_name.empty() && !ext.empty())
                {
                    if (basename == file_name && ext == file_ext)
                    {
                        m_file_list.push_back(cp.string());
                    }
                }
                else
                {
                    m_file_list.push_back(cp.string());
                }
            }
        }
        return m_file_list;
    }

    /**
     * @brief checkDirExists
     * If the directory does not exist, it is created.
     * @param path
     * @return true if the directory existed or was created, false otherwise.
     */
    static bool checkDirExists(string path)
    {
        const char* cpath = path.c_str();
        boost::filesystem::path dir(cpath);
        if (boost::filesystem::is_directory(dir))
        {
            return true;
        }
        if (!boost::filesystem::is_regular_file(dir))
        {
            return boost::filesystem::create_directory(dir);
        }
        return false;
    }

    static osg::Vec3d getHPRfromQuat(osg::Quat quat)
    {
        double qx = quat.x();
        double qy = quat.y();
        double qz = quat.z();
        double qw = quat.w();

        double sqx = qx * qx;
        double sqy = qy * qy;
        double sqz = qz * qz;
        double sqw = qw * qw;

        double term1 = 2*(qx*qy+qw*qz);
        double term2 = sqw+sqx-sqy-sqz;
        double term3 = -2*(qx*qz-qw*qy);
        double term4 = 2*(qw*qx+qy*qz);
        double term5 = sqw - sqx - sqy + sqz;

        double heading = atan2(term1, term2);
        double pitch = atan2(term4, term5);
        double roll = asin(term3);

        return osg::Vec3d( heading, pitch, roll );
    }

    static void toEulerAngle(const osg::Quat& q, double& roll, double& pitch, double& yaw)
    {
        //taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // roll (x-axis rotation)
        double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
        double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        roll = atan2(sinr, cosr);

        // pitch (y-axis rotation)
        double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = asin(sinp);

        // yaw (z-axis rotation)
        double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        yaw = atan2(siny, cosy);
    }

    /**
     * Taken from https://gist.github.com/yfnick/6ba33efa7ba12e93b148
     * @brief compress
     * @param data
     * @return
     */
    static std::string gz_compress(const std::string& data)
    {
        namespace bio = boost::iostreams;

        std::stringstream compressed;
        std::stringstream origin(data);

        bio::filtering_streambuf<bio::input> out;
        out.push(bio::gzip_compressor(bio::gzip_params(bio::gzip::best_compression)));
        out.push(origin);
        bio::copy(out, compressed);

        return compressed.str();
    }

    /**
     * Taken from https://gist.github.com/yfnick/6ba33efa7ba12e93b148
     * @brief compress
     * @param data
     * @return
     */
    static std::string gz_decompress(const std::string& data)
    {
        namespace bio = boost::iostreams;

        std::stringstream compressed(data);
        std::stringstream decompressed;

        bio::filtering_streambuf<bio::input> out;
        out.push(bio::gzip_decompressor());
        out.push(compressed);
        bio::copy(out, decompressed);

        return decompressed.str();
    }

    static void printMatrixToFile(string filename, const Eigen::MatrixXd mat)
    {
        fstream f(filename, f.out);
        f << mat;
        f << std::endl;
        f.close();
    }

    static void printMatrixToFile(string filename,
                                  const Eigen::Matrix4d mat,
                                  Eigen::Vector3d centroid)
    {
        Eigen::Matrix3d R = mat.block(0, 0, 3, 3);
        Eigen::Matrix4d m = mat;
        m.block(0, 3, 3, 1) = m.block(0, 3, 3, 1) - (-R * centroid);
        fstream f(filename, f.out);
        f << m;
        f << std::endl;
        f.close();
    }

    static vector<float> loadVectorFromStream(istream &f,
                                              int &width, int &height)
    {
        string line;
        std::vector<float> vec;
        width = 0;
        height = 0;
        while(std::getline(f, line))
        {
            size_t prev = 0;
            size_t next = 0;
            width = 0;
            while(next != string::npos)
            {
                next = line.find(" ", prev);
                if (next != string::npos)
                {
                    string token = line.substr(prev, next - prev);
                    prev = next + 1;
                    if (token == "")
                    {
                        continue;
                    }
                    vec.push_back(std::stof(token));
                    ++width;
                }
                else if (line.length() - prev > 0)
                {
                    string token = line.substr(prev, next - prev);
                    vec.push_back(std::stof(token));
                    ++width;
                }
            }
            ++height;
        }
        return vec;
    }

    static vector<double> loadDoubleVectorFromStream(istream &f,
                                              int &width, int &height)
    {
        string line;
        std::vector<double> vec;
        width = 0;
        height = 0;
        while(std::getline(f, line))
        {
            size_t prev = 0;
            size_t next = 0;
            width = 0;
            while(next != string::npos)
            {
                next = line.find(" ", prev);
                if (next != string::npos)
                {
                    string token = line.substr(prev, next - prev);
                    prev = next + 1;
                    if (token == "")
                    {
                        continue;
                    }
                    //std::cout << "token: `" << token << "`" << std::endl;
                    vec.push_back(std::stod(token));
                    ++width;
                }
                else if (line.length() - prev > 0)
                {
                    string token = line.substr(prev, next - prev);
                    //std::cout << "token: `" << token << "`" << std::endl;
                    vec.push_back(std::stod(token));
                    ++width;
                }
            }
            ++height;
        }
        return vec;
    }

    static Eigen::MatrixXd loadMatrixFromFile(string filename)
    {
        ifstream fstr(filename);
        int width, height;
        vector<double> vec = loadDoubleVectorFromStream(fstr, width, height);

        Eigen::MatrixXd mat;
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                mat(j, i) = vec[size_t(j * width + i)];
            }
        }
        return mat;
    }

    static Eigen::Matrix4d loadMatrix4dFromFile(string filename)
    {
        ifstream fstr(filename);
        int width, height;
        vector<double> vec = loadDoubleVectorFromStream(fstr, width, height);
        if (width == 4 && height == 4)
        {
            Eigen::Matrix4d mat;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    mat(j, i) = vec[size_t(j * width + i)];
                }
            }
            return mat;
        }
        else
        {
            throw std::runtime_error("Trying to load larger matrix than 4x4 \
                from file: " + filename + ", width: " + std::to_string(width) +
                ", height: " + std::to_string(height) + ".");
        }
    }

    static Eigen::Matrix4d loadMatrix4dFromFile(string filename,
                                                Eigen::Vector3d centroid)
    {
        Eigen::Matrix4d mat = loadMatrix4dFromFile(filename);
        Eigen::Matrix3d R = mat.block(0, 0, 3, 3);
        Eigen::Matrix4d m = mat;
        m.block(0, 3, 3, 1) = m.block(0, 3, 3, 1) - R * centroid;
        return m;
    }

    static Eigen::Vector3d loadSceneCenter(string filename)
    {
        ifstream sc_file(filename);
        std::string name;
        getline(sc_file, name, ':');
        if (name != "center")
        {
            throw runtime_error("Unable to parse scene center, "
                                "unknown format. Format must be center:x y z, "
                                "but the line started with " + name);
        }
        string token;
        int idx = 0;
        float xyz[3];
        while(idx < 3)
        {
            if (getline(sc_file, token, ' ').fail())
            {
                break;
            }
            try
            {
                xyz[idx] = stof(token);
                ++idx;
            }
            catch (std::invalid_argument &ia)
            {
                continue;
            }
        }
        if (idx == 3)
        {
            return Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
        }
        throw runtime_error("Unable to parse scene center, "
                            "unknown format. Format must be center:x y z, "
                            "last token `" + name + "` idx: "
                            + std::to_string(idx));
    }


    static Eigen::Matrix3d loadLocateRotationMatrixXML(string rotation_matrix_xml_file)
    {
        osg::ref_ptr<osgDB::XmlNode> xml = osgDB::readXmlFile(rotation_matrix_xml_file);
        Eigen::Matrix3d rot;

        if (xml->children.size() > 1)
        {
            osg::ref_ptr<osgDB::XmlNode> obj = xml->children[1];
            int colidx = 0;
            for (osg::ref_ptr<osgDB::XmlNode> col : obj->children)
            {
                std::string::size_type sz;
                std::string xs = col->properties.at("x0");
                double x0 = std::stod(xs, &sz);
                xs = col->properties.at("x1");
                double x1 = std::stod(xs, &sz);
                xs = col->properties.at("x2");
                double x2 = std::stod(xs, &sz);
                rot(0, colidx) = x0;
                rot(1, colidx) = x1;
                rot(2, colidx) = x2;
                ++colidx;
            }
        }
        return rot;
    }

    static void printLocateRotationMatrixXML(osg::Matrixd rot, string outpath)
    {
        osg::ref_ptr<osgDB::XmlNode> xml = new osgDB::XmlNode();
        xml->type = osgDB::XmlNode::ROOT;

        osg::ref_ptr<osgDB::XmlNode> object = new osgDB::XmlNode();
        object->name = "Object";
        object->type = osgDB::XmlNode::GROUP;

        for (int i = 0; i < 3; ++i)
        {
            osgDB::XmlNode *column = new osgDB::XmlNode();
            column->name = "C" + std::to_string(i);
            column->type = osgDB::XmlNode::ATOM;
            for (int j = 0; j < 3; ++j)
            {
                string cn = "x" + std::to_string(j);
                string val = std::to_string(rot(i, j));
                column->properties.insert(std::make_pair(cn, val));
            }
            object->children.push_back(column);
        }
        xml->children.push_back(object);

        // I did not find a better way to write the Doctype
        // there seems not to be a xml type for it
        // Some controls must be overriden so that the '<' is not changed
        // for '&lt;, etc.
        osgDB::XmlNode::ControlMap cmap;
        cmap.addControlToCharacter("<", '<');
        cmap.addControlToCharacter(">", '>');
        cmap.addControlToCharacter("\n", '\n');

        ofstream output(outpath);
        xml->writeString(cmap, output, "<!DOCTYPE ObjectXML>\n");
        xml->write(output);
        output.close();
    }

    static Eigen::MatrixXf loadDepthMap(string path)
    {
        string depth;
        ifstream f(path, ios::ate);
        if (f.is_open())
        {
            streampos size = f.tellg();
            char *data = new char[size];
            f.seekg(0, ios::beg);
            f.read(data, size);
            f.close();

            string depth_gz = string(data, static_cast<unsigned long>(size));
            depth = itr::Util::gz_decompress(depth_gz);
            std::stringstream ss(depth);
            int width = 0, height = 0;
            std::vector<float> depth_vec = loadVectorFromStream(ss, width, height);
            float *depth_arr = new float[width * height];
            std::copy(depth_vec.begin(), depth_vec.end(), depth_arr);
            Eigen::MatrixXf depth_mat = Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(depth_arr,
                                                             height, width);
            /*std::ofstream file("/Users/janbrejcha/Downloads/reprojected_image_depth.txt");
            if (file.is_open())
            {
                file << depth_mat << std::endl;
            }
            file.close();*/

            return depth_mat;
        }

        throw std::runtime_error("Unable to load depth image from path: "
                                 + path);
    }

    /**
     * @brief projectionToIntrinsics
     * Converts the projection matrix to intrinsics matrix.
     * @param proj the projection matrix to be converted
     * @param width width of the image plane in pixels.
     * @param height height of the image plane in pixels.
     * @return the intrisincs matrix.
     */
    static Eigen::Matrix3d projectionToIntrinsics(Eigen::Matrix4d proj,
                                                  int width, int height)
    {
        double f_w = proj(0, 0);
        double f_h = proj(1, 1);

        double cx = width / 2.0;
        double cy = height / 2.0;

        double fx = cx * f_w;
        double fy = cy * f_h;

        Eigen::Matrix3d intr;
        intr << fx, -0, -cx,
                0, -fy, -cy,
                0, -0, -1;

        std::cout << "intrinsics: \n" << intr << std::endl;
        return intr;
    }

    /**
     * @brief unproject_image
     * Unprojects the image coordinates into the 3D based on the modelview,
     * intrinsics and depth.
     * @param depth
     * @param modelview
     * @param intrinsics
     * @param stride defines the space between 2D points to be generated.
     * If stride = 1, then number of 2D points corresponds to number of pixels.
     * @return Eigen matrix Nx3 of 3D points.
     */
    static Eigen::MatrixX3d unproject_image(Eigen::MatrixXf depth,
                                            Eigen::Matrix4d modelview,
                                            Eigen::Matrix3d intrinsics,
                                            int stride = 100)
    {
        //create 2D coordinates
        //modelview.block(1, 0, 2, 4) = -modelview.block(1, 0, 2, 4);
        long numpts = (depth.cols() * depth.rows()) / stride;

        std::vector<Eigen::Vector3d> coords_2d_vec;
        Eigen::MatrixXd depth_d = depth.cast<double>();
        long idx = 0;
        for (int i = 0; i < depth.cols(); i += stride)
        {
            for (int j = 0; j < depth.rows(); j += stride)
            {
                double depth = depth_d(j, i);
                if (depth < 500000)
                {
                    coords_2d_vec.push_back(Eigen::Vector3d(i, j, 1) * depth);
                }
               ++idx;
            }
        }

        numpts = static_cast<long>(coords_2d_vec.size());
        Eigen::MatrixX3d coords_2d(numpts, 3);
        for (long i = 0; i < numpts; ++i)
        {
            coords_2d.row(i) = coords_2d_vec[i];
        }

        Eigen::Matrix3d intr_inv = intrinsics.inverse();
        Eigen::MatrixX3d coords_3d = (intr_inv * coords_2d.transpose()).transpose();
        Eigen::MatrixX4d coords_3d_world(numpts, 4);
        coords_3d_world.block(0, 0, numpts, 3) = coords_3d;
        coords_3d_world.block(0, 3, numpts, 1) = Eigen::MatrixXd::Constant(numpts, 1, 1);

        coords_3d_world = (modelview.inverse() * coords_3d_world.transpose()).transpose();
        coords_3d.block(0, 0, numpts, 3) = coords_3d_world.block(0, 0, numpts, 3);

        return coords_3d;
    }

    static Eigen::MatrixX2d project(Eigen::MatrixX3d pt_3d,
                                    Eigen::Matrix4d modelview,
                                    Eigen::Matrix3d intrinsics)
    {
        int numpts = pt_3d.rows();
        Eigen::MatrixX4d pt_4d(numpts, 4);
        pt_4d.block(0, 0, numpts, 3) = pt_3d;
        pt_4d.block(0, 3, numpts, 1) = Eigen::MatrixXd::Constant(numpts, 1, 1);
        pt_4d = pt_4d * modelview.transpose();
        //divide by 4th column
        Eigen::VectorXd w = pt_4d.block(0, 3, numpts, 1);
        pt_4d = (pt_4d.array().colwise() / w.array()).matrix();
        pt_3d = pt_4d.block(0, 0, numpts, 3) * intrinsics.transpose();
        Eigen::MatrixX2d pt_2d(numpts, 2);
        Eigen::VectorXd z = pt_3d.block(0, 2, numpts, 1);
        pt_2d = (pt_3d.array().colwise() / z.array()).matrix().block(0, 0, numpts, 2);
        return pt_2d;
    }


    /**
     * @brief The normal_random_variable struct
     * Taken from https://stackoverflow.com/a/40245513/2077284
     */
    struct normal_random_variable
    {
        normal_random_variable(Eigen::MatrixXd const& covar)
            : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
        {}

        normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
            : mean(mean)
        {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
            transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
        }

        Eigen::VectorXd mean;
        Eigen::MatrixXd transform;

        Eigen::VectorXd operator()() const
        {
            static std::mt19937 gen{ std::random_device{}() };
            static std::normal_distribution<> dist;

            return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](double x) { return dist(gen); });
        }
    };

};

}

#endif // UTIL_H
