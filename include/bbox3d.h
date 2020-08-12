/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   21.08.2017
* @Email:  ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
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


#ifndef BBOX3D_H
#define BBOX3D_H

//Eigen headers
#include <Eigen/Dense>

//stl headers
#include <limits>

using namespace Eigen;
using namespace std;


namespace itr
{
    /**
     * @brief The BBox3D struct
     * Stores the 3D bounding box and provides utility methods for analysis and
     * manipulation of this 3D box.
     */
    struct BBox3D
    {
        /// bottom-front-left coordinate
        Vector3d min;

        /// top-back-right coordinate
        Vector3d max;

        /**
         * @brief BBox3D
         * Default initializer, initializes min coordinate with maximum double
         * number and min coordinate with minimum double number so that you can
         * sequentially search for the bounding box looping over geometry.
         */
        BBox3D();

        /**
         * @brief getDimensions
         * Calculates dimensions of the bounding box.
         * @return dimensions of the bounding box.
         */
        Vector3d calculateDimensions();

    };
}

#endif // BBOX3D_H
