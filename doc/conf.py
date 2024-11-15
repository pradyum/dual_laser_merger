# Copyright 2024 pradyum
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys


sys.path.insert(0, os.path.abspath('.'))

project = 'dual_lidar_merger'
author = 'Pradyum Aadith'
copyright = '2024 pradyum'
version = '1.0.0'
release = '1.0.0'
language = 'en'
root_doc = 'index'

html_static_path = ['_static']

def setup(app):
    app.add_css_file('page.css')