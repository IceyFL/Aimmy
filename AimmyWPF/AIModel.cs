using KdTree;
using KdTree.Math;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AimmyAimbot
{
    public static class Win32API
    {
        public const int SRCCOPY = 0x00CC0020;

        [DllImport("user32.dll")]
        public static extern IntPtr GetDC(IntPtr hwnd);

        [DllImport("user32.dll")]
        public static extern int ReleaseDC(IntPtr hwnd, IntPtr hdc);

        [DllImport("gdi32.dll")]
        public static extern bool BitBlt(IntPtr hdcDest, int nXDest, int nYDest, int nWidth, int nHeight, IntPtr hdcSrc, int nXSrc, int nYSrc, int dwRop);
    }


    public class AIModel : IDisposable
    {
        private const int IMAGE_SIZE = 640;
        private const int NUM_DETECTIONS = 8400; // Standard for OnnxV8 model (Shape: 1x5x8400)

        private readonly RunOptions _modeloptions;
        private InferenceSession _onnxModel;

        public float ConfidenceThreshold = 0.6f;
        public bool CollectData = false;
        public int FovSize = 640;

        private DateTime lastSavedTime = DateTime.MinValue;
        private List<string> _outputNames;

        private Bitmap _screenCaptureBitmap = new Bitmap(IMAGE_SIZE, IMAGE_SIZE); // Initialize _screenCaptureBitmap with a default size


        // Image size will always be 640x640
        private static byte[] _rgbValuesCache = new byte[640 * 640 * 3];

        public AIModel(string modelPath)
        {
            _modeloptions = new RunOptions();

            var sessionOptions = new SessionOptions
            {
                EnableCpuMemArena = true,
                EnableMemoryPattern = true,
                GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL,
                ExecutionMode = ExecutionMode.ORT_PARALLEL
            };

            // Attempt to load via DirectML (else fallback to CPU)
            try
            {
                LoadViaDirectML(sessionOptions, modelPath);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"There was an error starting the OnnxModel via DirectML: {ex}\n\nProgram will attempt to use CPU only, performance may be poor.", "Model Error");
                LoadViaCPU(sessionOptions, modelPath);
            }

            // Validate the onnx model output shape (ensure model is OnnxV8)
            ValidateOnnxShape();
        }

        private void LoadViaDirectML(SessionOptions sessionOptions, string modelPath)
        {
            sessionOptions.AppendExecutionProvider_DML();
            _onnxModel = new InferenceSession(modelPath, sessionOptions);
            _outputNames = _onnxModel.OutputMetadata.Keys.ToList();
        }

        private void LoadViaCPU(SessionOptions sessionOptions, string modelPath)
        {
            try
            {
                sessionOptions.AppendExecutionProvider_CPU();
                _onnxModel = new InferenceSession(modelPath, sessionOptions);
                _outputNames = _onnxModel.OutputMetadata.Keys.ToList();
            }
            catch (Exception e)
            {
                MessageBox.Show($"Error starting the model via CPU: {e}");
                System.Windows.Application.Current.Shutdown();
            }
        }

        private void ValidateOnnxShape()
        {
            foreach (var output in _onnxModel.OutputMetadata)
            {
                var shape = _onnxModel.OutputMetadata[output.Key].Dimensions;
                if (shape.Length != 3 || shape[0] != 1 || shape[1] != 5 || shape[2] != NUM_DETECTIONS)
                {
                    MessageBox.Show($"Output shape {string.Join("x", shape)} does not match the expected shape of 1x5x8400.\n\nThis model will not work with Aimmy, please use an ONNX V8 model.", "Model Error");
                }
            }
        }

        public class Prediction
        {
            public RectangleF Rectangle { get; set; }
            public float Confidence { get; set; }
        }

        public static float AIConfidence { get; set; }

        public Bitmap ScreenGrab(Rectangle detectionBox)
        {
            // Ensure a bitmap exists and matches the required size
            if (_screenCaptureBitmap == null ||
                _screenCaptureBitmap.Width != detectionBox.Width ||
                _screenCaptureBitmap.Height != detectionBox.Height)
            {
                // Dispose of the existing bitmap if needed
                _screenCaptureBitmap?.Dispose();

                // Create a new bitmap with the required size
                _screenCaptureBitmap = new Bitmap(detectionBox.Width, detectionBox.Height);
            }

            // Use direct memory copy with the specific area defined
            using (Graphics g = Graphics.FromImage(_screenCaptureBitmap))
            {
                IntPtr hdc = g.GetHdc(); // Get the device context handle
                try
                {
                    // Capture the screen directly into the bitmap's device context
                    Win32API.BitBlt(hdc, 0, 0, detectionBox.Width, detectionBox.Height,
                                    Win32API.GetDC(IntPtr.Zero), detectionBox.Left, detectionBox.Top,
                                    Win32API.SRCCOPY);
                }
                finally
                {
                    g.ReleaseHdc(hdc); // Release the device context handle
                }
            }

            return _screenCaptureBitmap;
        }

        public static float[] BitmapToFloatArray(Bitmap image)
        {
            int height = image.Height;
            int width = image.Width;
            int stride;
            float[] result = new float[3 * height * width];

            BitmapData bmpData = image.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
            unsafe
            {
                byte* ptr = (byte*)bmpData.Scan0;
                stride = bmpData.Stride;

                Parallel.For(0, height, y =>
                {
                    int rowIndex = y * stride;

                    for (int x = 0; x < width; x++)
                    {
                        int index = rowIndex + x * 3;

                        float r = ptr[index + 2] / 255.0f; // R
                        float g = ptr[index + 1] / 255.0f; // G
                        float b = ptr[index] / 255.0f; // B

                        int resultIndex = y * width + x;
                        result[resultIndex] = r;
                        result[resultIndex + height * width] = g;
                        result[resultIndex + 2 * height * width] = b;
                    }
                });
            }
            image.UnlockBits(bmpData);

            return result;
        }

        public async Task<Prediction> GetClosestPredictionToCenterAsync(int aimmethod)
        {
            // Define the detection box
            int halfScreenWidth = Screen.PrimaryScreen.Bounds.Width / 2;
            int halfScreenHeight = Screen.PrimaryScreen.Bounds.Height / 2;
            int detectionBoxSize = 640;
            Rectangle detectionBox = new Rectangle(halfScreenWidth - detectionBoxSize / 2,
                                                   halfScreenHeight - detectionBoxSize / 2,
                                                   detectionBoxSize,
                                                   detectionBoxSize);

            // Capture a screenshot
            Bitmap frame = ScreenGrab(detectionBox); ;

            // Convert the Bitmap to float array and normalize
            float[] inputArray = BitmapToFloatArray(frame);
            if (inputArray == null) { return null; }

            Tensor<float> inputTensor = new DenseTensor<float>(inputArray, new int[] { 1, 3, frame.Height, frame.Width });
            var inputs = new List<NamedOnnxValue> { NamedOnnxValue.CreateFromTensor("images", inputTensor) };
            var results = _onnxModel.Run(inputs, _outputNames, _modeloptions);

            var outputTensor = results[0].AsTensor<float>();

            // Calculate the FOV boundaries
            float fovMinX = (IMAGE_SIZE - FovSize) / 2.0f;
            float fovMaxX = (IMAGE_SIZE + FovSize) / 2.0f;
            float fovMinY = (IMAGE_SIZE - FovSize) / 2.0f;
            float fovMaxY = (IMAGE_SIZE + FovSize) / 2.0f;

            var tree = new KdTree<float, Prediction>(2, new FloatMath());

            var filteredIndices = Enumerable.Range(0, NUM_DETECTIONS)
                                    .AsParallel()
                                    .Where(i => outputTensor[0, 4, i] >= ConfidenceThreshold)
                                    .ToList();

            object treeLock = new object();

            Parallel.ForEach(filteredIndices, i =>
            {
                float objectness = outputTensor[0, 4, i];
                AIConfidence = objectness;

                float x_center = outputTensor[0, 0, i];
                float y_center = outputTensor[0, 1, i];
                float width = outputTensor[0, 2, i];
                float height = outputTensor[0, 3, i];

                float x_min = x_center - width / 2;
                float y_min = y_center - height / 2;
                float x_max = x_center + width / 2;
                float y_max = y_center + height / 2;

                if (x_min >= fovMinX && x_max <= fovMaxX && y_min >= fovMinY && y_max <= fovMaxY)
                {
                    var prediction = new Prediction
                    {
                        Rectangle = new RectangleF(x_min, y_min, x_max - x_min, y_max - y_min),
                        Confidence = objectness
                    };

                    var centerX = (x_min + x_max) / 2.0f;
                    var centerY = (y_min + y_max) / 2.0f;

                    lock (tree)
                    {
                        tree.Add(new[] { centerX, centerY }, prediction);
                    }
                }
            });

            // Collect predictions from the tree
            var predictions = new List<Prediction>();
            lock (tree)
            {
                foreach (var node in tree)
                {
                    predictions.Add(node.Value);
                }
            }

            foreach (var i in filteredIndices)
            {
                float objectness = outputTensor[0, 4, i];
                AIConfidence = objectness;

                float x_center = outputTensor[0, 0, i];
                float y_center = outputTensor[0, 1, i];
                float width = outputTensor[0, 2, i];
                float height = outputTensor[0, 3, i];

                float x_min = x_center - width / 2;
                float y_min = y_center - height / 2;
                float x_max = x_center + width / 2;
                float y_max = y_center + height / 2;

                if (x_min >= fovMinX && x_max <= fovMaxX && y_min >= fovMinY && y_max <= fovMaxY)
                {
                    var prediction = new Prediction
                    {
                        Rectangle = new RectangleF(x_min, y_min, x_max - x_min, y_max - y_min),
                        Confidence = objectness
                    };

                    var centerX = (x_min + x_max) / 2.0f;
                    var centerY = (y_min + y_max) / 2.0f;

                    lock (treeLock)
                    {
                        tree.Add(new[] { centerX, centerY }, prediction);
                    }
                    predictions.Add(prediction);
                }
            }

            // Querying the KDTree for the closest prediction to the center.
            var nodes = tree.GetNearestNeighbours(new[] { IMAGE_SIZE / 2.0f, IMAGE_SIZE / 2.0f }, 1);

            // Save frame asynchronously if the option is turned on
            if (CollectData)
            {
                DateTime currentTime = DateTime.Now;
                if ((currentTime - lastSavedTime).TotalSeconds >= 0.5)
                {
                    lastSavedTime = currentTime;
                    string uuid = Guid.NewGuid().ToString();
                    if (nodes.Length > 0)
                    {
                        await Task.Run(() => frame.Save($"bin/images/{uuid}.jpg"));
                        var closestPrediction = nodes[0].Value;

                        // Create and write to the label file
                        string labelFilePath = $"bin/labels/{uuid}.txt";
                        using (StreamWriter writer = new StreamWriter(labelFilePath))
                        {
                            // Convert coordinates to YOLO format and write to the file
                            float x = closestPrediction.Rectangle.X / IMAGE_SIZE;
                            float y = closestPrediction.Rectangle.Y / IMAGE_SIZE;
                            float width = closestPrediction.Rectangle.Width / IMAGE_SIZE;
                            float height = closestPrediction.Rectangle.Height / IMAGE_SIZE;

                            writer.WriteLine($"0 {x + width / 2} {y + height / 2} {width} {height}");
                        }
                    }
                }
            }

            if (aimmethod == 0)
            {
                return nodes.Length > 0 ? nodes[0].Value : (Prediction?)null;
            }
            else
            {
                var highestConfidencePrediction = predictions.OrderByDescending(p => p.Confidence).FirstOrDefault();
                return highestConfidencePrediction;
            }
        }

        public void Dispose()
        {
            _onnxModel?.Dispose();
            _screenCaptureBitmap?.Dispose();
            GC.SuppressFinalize(this); // called to prevent the finalizer from running if the object is already disposed of.
        }
    }
}