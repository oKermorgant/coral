#include <coral/water_lights.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <coral/weather.h>

using namespace coral;

using Table = osg::Vec3f[64];

auto interp(const Table &table, float val)
{
  val = std::clamp(val, 0.f, 1.f) * 63;
  const size_t idx = val;
  const float ratio{val-idx};
  if(ratio > 0.)
    return table[idx]*(1+ratio) + table[idx+1]*(1-ratio);
  return table[idx];
}


void Water::fromJerlov(float val)
{
  from(Weather::Mood::CLEAR);

  // TODO sync with rendering from Stonefish simulator
  static bool init{false};
  static Table absorption, scattering;
  if(!init)
  {
    init = true;
    const auto data{ament_index_cpp::get_package_share_directory("coral") + "/resources/jerlov.dat"};
    std::ifstream dataFile(data, std::ios::in | std::ios::binary);
    dataFile.read((char*)absorption, sizeof(absorption));
    dataFile.read((char*)scattering, sizeof(scattering));
    dataFile.close();
  }

  /*

  diffuse = Weather::intColor(72, 110, 123);
  //diffuse = osg::Vec4f(interp(scattering,val), 1.f);
  //diffuse.normalize();
  fogColor = diffuse;
  //attenuation = (interp(absorption,val) + interp(scattering, val))/100;
  attenuation = interp(absorption,val)/10;

  std::cout << "attenuation is now " << attenuation.x() << " " << attenuation.y() << " " << attenuation.z() << std::endl;

  */

}

void Water::from(Weather::Mood mood)
{
  switch (mood)
  {
    case Weather::Mood::CLEAR:
      attenuation = osg::Vec3f(0.015f, 0.0075f, 0.005f);
      diffuse = Weather::intColor(27,57,109);
      fogColor = Weather::intColor(72,110,123);
      fogDensity = 0.015;
      break;
    case Weather::Mood::DUSK:
      attenuation =  osg::Vec3f(0.015f, 0.0075f, 0.005f);
      diffuse = Weather::intColor(44,69,106);
      fogColor = Weather::intColor(44,69,106);
      fogDensity = 0.03;
      break;
    case Weather::Mood::CLOUDY:
      attenuation = osg::Vec3f(0.008f, 0.003f, 0.002f);
      diffuse = Weather::intColor(84,135,172);
      fogColor = Weather::intColor(28,80,100);
      fogDensity = 0.02;
      break;
    case Weather::Mood::NIGHT:
      attenuation = osg::Vec3f(0.8f, 0.3f, 0.2f);
      fogColor = diffuse = Weather::intColor(10, 10, 30);
      fogDensity = 0.06;
      break;
    default:
      break;
  }
}
