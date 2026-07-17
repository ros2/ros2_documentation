#VRML_SIM R2022b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/floors/protos/CircleArena.proto"

WorldInfo {
}
Viewpoint {
  orientation -0.33185733874619844 -0.09874274160469809 0.9381474178937331 3.686018050088086
  position 1.700313773507203 1.0549607538959629 1.4846240848267684
  follow "my_robot"
}
TexturedBackground {
}
TexturedBackgroundLight {
}
CircleArena {
}
Robot {
  children [
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0.025
      }
      device [
        RotationalMotor {
          name "left wheel motor"
        }
      ]
      endPoint Solid {
        translation 0 0.045 0.025
        children [
          DEF WHEEL Transform {
            rotation 1 0 0 1.5707996938995747
            children [
              Shape {
                appearance PBRAppearance {
                  baseColor 1 0 0
                  roughness 1
                  metalness 0
                }
                geometry Cylinder {
                  height 0.01
                  radius 0.025
                }
              }
            ]
          }
        ]
        name "left wheel"
        boundingObject USE WHEEL
        physics Physics {
        }
      }
    }
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0.025
      }
      device [
        RotationalMotor {
          name "right wheel motor"
        }
      ]
      endPoint Solid {
        translation 0 -0.045 0.025
        children [
          USE WHEEL
        ]
        name "right wheel"
        boundingObject USE WHEEL
        physics Physics {
        }
      }
    }
    Transform {
      translation 0 0 0.0415
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 0 0 1
            roughness 1
            metalness 0
          }
          geometry DEF BODY Cylinder {
            height 0.08
            radius 0.045
          }
        }
      ]
    }
    DistanceSensor {
      translation 0.042 0.02 0.063
      rotation 0 0 1 0.5236003061004253
      children [
        DEF SENSOR Transform {
          rotation 0 1 0 1.5708
          children [
            Shape {
              appearance PBRAppearance {
                baseColor 1 1 0
                roughness 1
                metalness 0
              }
              geometry Cylinder {
                height 0.004
                radius 0.008
              }
            }
          ]
        }
      ]
      name "ds0"
      lookupTable [
        0 1020 0
        0.05 1020 0
        0.15 0 0
      ]
      numberOfRays 2
      aperture 1
    }
    DistanceSensor {
      translation 0.042 -0.02 0.063
      rotation 0 0 1 -0.5235996938995747
      children [
        USE SENSOR
      ]
      name "ds1"
      lookupTable [
        0 1020 0
        0.05 1020 0
        0.15 0 0
      ]
      numberOfRays 2
      aperture 1
    }
  ]
  boundingObject Transform {
    translation 0 0 0.0415
    children [
      USE BODY
    ]
  }
  physics Physics {
  }
  controller "<extern>"
  name "my_robot"
}
