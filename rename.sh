#!/bin/bash
mv include/psen_scan/ include/psen_scan_v2
mv test/include/psen_scan/ test/include/psen_scan_v2

# Replace ifndef-guards in header-files
find . -mindepth 2 \( -type d -name .git -prune \) -o -name *.h -print0 | xargs -0 sed -i -E 's/PSEN_SCAN(_.*_H)/PSEN_SCAN_V2\1/g'

# Replace includes and namespaces in cpp- and header-files
find . -mindepth 2 \( -type d -name .git -prune \) -o -name "*.test" -print0 | xargs -0 sed -i 's/psen_scan/psen_scan_v2/g'
find . -mindepth 2 \( -type d -name .git -prune \) -o -name "*.h" -print0 | xargs -0 sed -i 's/psen_scan/psen_scan_v2/g'
find . -mindepth 2 \( -type d -name .git -prune \) -o -name "*.cpp" -print0 | xargs -0 sed -i 's/psen_scan/psen_scan_v2/g'

# Rest
sed -i -E 's/psen_scan([^_\.])/psen_scan_v2\1/g' CMakeLists.txt launch/psen_scan.launch test/acceptance_tests/acceptancetest_psen_scan.md
sed -i -E 's/psen_scan(_coverage)/psen_scan_v2\1/g' CMakeLists.txt
sed -i -E 's/psen_scan/psen_scan_v2/g' package.xml
sed -i -E 's/psen_scan([^_\.\)]|$)/psen_scan_v2\1/g' README.md