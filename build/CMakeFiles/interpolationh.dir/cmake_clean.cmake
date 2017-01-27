file(REMOVE_RECURSE
  "libinterpolationh.pdb"
  "libinterpolationh.dylib"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/interpolationh.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
