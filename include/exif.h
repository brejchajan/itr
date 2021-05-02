/**
* @Author: Jan Brejcha <janbrejcha>
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



#ifndef EXIF_H
#define EXIF_H

#include <string>
#include <limits>

#include "third_party/easyexif/exif.h"
#include "openMVG/exif/exif_IO_EasyExif.hpp"

namespace itr{

using namespace std;

class Exif
{
public:
    Exif()
        :bHaveExifInfo(false)
    {}

    Exif(const string &fileName)
    {
        open(fileName);
    }

    string getDateTimeOriginal()
    {
        return exifInfo.DateTimeOriginal;
    }

    string getDateTimeDigitized()
    {
        return exifInfo.DateTimeDigitized;
    }

    string getDateTime()
    {
        return exifInfo.DateTime;
    }

    bool GPSLatitude(double * latitude) const
    {
        if (exifInfo.GeoLocation.Latitude != std::numeric_limits<double>::infinity())
        {
            (*latitude) = exifInfo.GeoLocation.Latitude;
            return true;
        }
        return false;
    }

    bool GPSLongitude(double * longitude) const
    {
        if (exifInfo.GeoLocation.Longitude != std::numeric_limits<double>::infinity())
        {
            (*longitude) = exifInfo.GeoLocation.Longitude;
            return true;
        }
        return false;
    }

private:
    easyexif::EXIFInfo exifInfo;
    bool bHaveExifInfo;

    /**
    * @brief Open and populate EXIF data
    * @param sFileName path of the image to analyze
    * @retval true if image file could be parsed correctly
    * @retval false if image file could not be parsed and analysed
    */
    bool open( const std::string & sFileName )
    {
      // Read the file into a buffer
      FILE *fp = fopen( sFileName.c_str(), "rb" );
      if ( !fp )
      {
        return false;
      }
      fseek( fp, 0, SEEK_END );
      unsigned long fsize = ftell( fp );
      rewind( fp );
      std::vector<unsigned char> buf( fsize );
      if ( fread( &buf[0], 1, fsize, fp ) != fsize )
      {
        fclose( fp );
        return false;
      }
      fclose( fp );

      // Parse EXIF
      bHaveExifInfo = ( exifInfo.parseFrom( &buf[0], fsize ) == PARSE_EXIF_SUCCESS );

      return bHaveExifInfo;
    }


};

} //namespace itr

#endif // EXIF_H
