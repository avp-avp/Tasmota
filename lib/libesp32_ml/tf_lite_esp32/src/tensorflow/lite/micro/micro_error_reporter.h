/* Copyright 2018 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#ifndef TENSORFLOW_LITE_MICRO_MICRO_ERROR_REPORTER_H_
#define TENSORFLOW_LITE_MICRO_MICRO_ERROR_REPORTER_H_

#include <cstdarg>

#include "tensorflow/lite/core/api/error_reporter.h"
#include "tensorflow/lite/micro/compatibility.h"
// TODO(b/246776144): Move this include statement to the cc file.
#include "tensorflow/lite/micro/micro_log.h"

namespace tflite {
// Get a pointer to a singleton global error reporter.
ErrorReporter* GetMicroErrorReporter();
class MicroErrorReporter : public ErrorReporter {
 public:
  ~MicroErrorReporter() override {}
  int Report(const char* format, va_list args) override;

 private:
  TF_LITE_REMOVE_VIRTUAL_DELETE
};

}  // namespace tflite

#endif  // TENSORFLOW_LITE_MICRO_MICRO_ERROR_REPORTER_H_
