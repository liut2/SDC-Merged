file(REMOVE_RECURSE
  "libstdafx.pdb"
  "libstdafx.dylib"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/stdafx.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
