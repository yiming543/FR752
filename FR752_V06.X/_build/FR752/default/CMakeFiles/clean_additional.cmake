# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "D:\\yiming\\00work\\FR752_V05.X\\out\\FR752\\default.cmf"
  "D:\\yiming\\00work\\FR752_V05.X\\out\\FR752\\default.hex"
  "D:\\yiming\\00work\\FR752_V05.X\\out\\FR752\\default.hxl"
  "D:\\yiming\\00work\\FR752_V05.X\\out\\FR752\\default.mum"
  "D:\\yiming\\00work\\FR752_V05.X\\out\\FR752\\default.o"
  "D:\\yiming\\00work\\FR752_V05.X\\out\\FR752\\default.sdb"
  "D:\\yiming\\00work\\FR752_V05.X\\out\\FR752\\default.sym"
  )
endif()
