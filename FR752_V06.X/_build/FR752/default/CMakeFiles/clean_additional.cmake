# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "D:\\yiming\\04github\\FR752\\FR752_V06.X\\out\\FR752\\default.cmf"
  "D:\\yiming\\04github\\FR752\\FR752_V06.X\\out\\FR752\\default.hex"
  "D:\\yiming\\04github\\FR752\\FR752_V06.X\\out\\FR752\\default.hxl"
  "D:\\yiming\\04github\\FR752\\FR752_V06.X\\out\\FR752\\default.mum"
  "D:\\yiming\\04github\\FR752\\FR752_V06.X\\out\\FR752\\default.o"
  "D:\\yiming\\04github\\FR752\\FR752_V06.X\\out\\FR752\\default.sdb"
  "D:\\yiming\\04github\\FR752\\FR752_V06.X\\out\\FR752\\default.sym"
  )
endif()
