Import("env")


env.Append(CXXFLAGS=["-Wno-register"])




env.Append(
  LINKFLAGS=[
      "-mfloat-abi=hard"
  ]
)

