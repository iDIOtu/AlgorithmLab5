using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;

namespace AlgorithmLab5
{
    public partial class MainWindow : Window
    {
        private Graph graph = new Graph();
        private Node selectedNode;
        private Edge hoveredEdge;
        private Node hoveredNode; // Добавляем переменную для отслеживания наведения на вершину
        private int _speed = 1000;

        public MainWindow()
        {
            InitializeComponent();
            GraphCanvas.MouseMove += GraphCanvas_MouseMove;
            this.KeyDown += MainWindow_KeyDown;
        }

        private void GraphCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point clickPosition = e.GetPosition(GraphCanvas);
            Node newNode = graph.AddNode("Vertex" + (graph.Nodes.Count + 1), clickPosition);
            DrawNode(newNode);
        }

        private void GraphCanvas_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point clickPosition = e.GetPosition(GraphCanvas);
            Node clickedNode = GetNodeAtPosition(clickPosition);

            if (selectedNode == null && clickedNode != null)
            {
                selectedNode = clickedNode;
                selectedNode.SetSelected(true);
            }
            else if (selectedNode != null && clickedNode != null && selectedNode != clickedNode)
            {
                Edge newEdge = graph.AddEdge(selectedNode, clickedNode, 1); // Default weight
                DrawEdge(newEdge);
                selectedNode.SetSelected(false);
                selectedNode = null;
            }
            else if (selectedNode != null)
            {
                selectedNode.SetSelected(false);
                selectedNode = null;
            }
        }

        private void GraphCanvas_MouseMove(object sender, MouseEventArgs e)
        {
            Point mousePosition = e.GetPosition(GraphCanvas);
            hoveredEdge = GetEdgeAtPosition(mousePosition);
            hoveredNode = GetNodeAtPosition(mousePosition); // Обновляем hoveredNode
        }

        private void MainWindow_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Delete)
            {
                if (hoveredEdge != null)
                {
                    graph.RemoveEdge(hoveredEdge);
                    UpdateGraphCanvas();
                }
                else if (hoveredNode != null) // Проверяем, если курсор над вершиной
                {
                    graph.RemoveNode(hoveredNode);
                    UpdateGraphCanvas();
                }
            }
        }

        private void ClearButton_Click(object sender, RoutedEventArgs e)
        {
            graph.Clear();
            GraphCanvas.Children.Clear();
        }

        private void SaveButton_Click(object sender, RoutedEventArgs e)
        {
            SaveFileDialog saveFileDialog = new SaveFileDialog
            {
                Filter = "CSV files (*.csv)|*.csv|All files (*.*)|*.*",
                Title = "Save Graph"
            };

            if (saveFileDialog.ShowDialog() == true)
            {
                graph.SaveToFile(saveFileDialog.FileName);
            }
        }

        private void LoadButton_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog openFileDialog = new OpenFileDialog
            {
                Filter = "CSV files (*.csv)|*.csv|All files (*.*)|*.*",
                Title = "Load Graph"
            };

            if (openFileDialog.ShowDialog() == true)
            {
                ClearButton_Click(sender, e);
                try
                {
                    graph.LoadFromFile(openFileDialog.FileName);
                    foreach (var node in graph.Nodes)
                    {
                        DrawNode(node);
                    }
                    foreach (var edge in graph.Edges)
                    {
                        DrawEdge(edge);
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error loading file: " + ex.Message);
                }
            }
        }

        private void DrawNode(Node node)
        {
            Ellipse ellipse = new Ellipse
            {
                Width = 60,
                Height = 60,
                Fill = Brushes.Blue,
                StrokeThickness = 2,
                Stroke = Brushes.Black,
                Tag = node
            };

            Canvas.SetLeft(ellipse, node.Position.X);
            Canvas.SetTop(ellipse, node.Position.Y);
            GraphCanvas.Children.Add(ellipse);

            TextBox textBox = new TextBox
            {
                Width = 50,
                Height = 20,
                Text = node.Name,
                Tag = node,
                Background = Brushes.White,
                BorderThickness = new Thickness(2),
                BorderBrush = Brushes.Black,
                VerticalAlignment = VerticalAlignment.Top,
                HorizontalAlignment = HorizontalAlignment.Center
            };

            Canvas.SetLeft(textBox, node.Position.X + 5);
            Canvas.SetTop(textBox, node.Position.Y + 20);
            GraphCanvas.Children.Add(textBox);

            textBox.TextChanged += (s, e) =>
            {
                node.Name = textBox.Text;
            };

            ellipse.MouseRightButtonDown += (s, e) =>
            {
                node.SetSelected(true);
            };

            // Добавляем обработчик для наведения на вершину
            ellipse.MouseEnter += (s, e) =>
            {
                hoveredNode = node; // Устанавливаем hoveredNode на текущее узел
            };

            ellipse.MouseLeave += (s, e) =>
            {
                if (hoveredNode == node)
                {
                    hoveredNode = null; // Убираем hoveredNode, если мышь покинула узел
                }
            };
        }

        private void DrawEdge(Edge edge)
        {
            Line line = new Line
            {
                X1 = edge.Start.Position.X + 30,
                Y1 = edge.Start.Position.Y + 30,
                X2 = edge.End.Position.X + 30,
                Y2 = edge.End.Position.Y + 30,
                Stroke = Brushes.Black,
                StrokeThickness = 2,
                Tag = edge // Добавляем тег для доступа к ребру
            };
            GraphCanvas.Children.Insert(0, line);

            TextBox weightTextBox = new TextBox
            {
                Width = 30,
                Height = 20,
                Text = edge.Weight.ToString(),
                Tag = edge,
                Background = Brushes.White,
                BorderThickness = new Thickness(1),
                BorderBrush = Brushes.Black,
                VerticalAlignment = VerticalAlignment.Top,
                HorizontalAlignment = HorizontalAlignment.Center
            };

            double midX = (edge.Start.Position.X + edge.End.Position.X) / 2 + 30;
            double midY = (edge.Start.Position.Y + edge.End.Position.Y) / 2 + 30;
            Canvas.SetLeft(weightTextBox, midX);
            Canvas.SetTop(weightTextBox, midY);
            GraphCanvas.Children.Add(weightTextBox);

            weightTextBox.TextChanged += (s, e) =>
            {
                if (int.TryParse(weightTextBox.Text, out int weight))
                {
                    edge.Weight = weight;
                }
            };

            // Обработчик события MouseMove для текстового поля веса
            weightTextBox.MouseEnter += (s, e) =>
            {
                hoveredEdge = edge; // Устанавливаем hoveredEdge на текущее ребро
            };

            weightTextBox.MouseLeave += (s, e) =>
            {
                if (hoveredEdge == edge)
                {
                    hoveredEdge = null; // Убираем hoveredEdge, если мышь покинула текстовое поле
                }
            };
        }

        private Node GetNodeAtPosition(Point position)
        {
            foreach (var child in GraphCanvas.Children)
            {
                if (child is Ellipse ellipse && ellipse.Tag is Node node)
                {
                    Rect bounds = new Rect(Canvas.GetLeft(ellipse), Canvas.GetTop(ellipse), ellipse.Width, ellipse.Height);
                    if (bounds.Contains(position))
                    {
                        return node;
                    }
                }
            }
            return null;
        }

        private Edge GetEdgeAtPosition(Point position)
        {
            foreach (var child in GraphCanvas.Children)
            {
                if (child is Line line && line.Tag is Edge edge)
                {
                    // Проверка, находится ли курсор в пределах линии
                    if (IsPointNearLine(position, line.X1, line.Y1, line.X2, line.Y2))
                    {
                        return edge;
                    }
                }
            }
            return null;
        }

        private bool IsPointNearLine(Point point, double x1, double y1, double x2, double y2, double tolerance = 5)
        {
            double distance = Math.Abs((y2 - y1) * point.X - (x2 - x1) * point.Y + x2 * y1 - y2 * x1) /
                              Math.Sqrt(Math.Pow(y2 - y1, 2) + Math.Pow(x2 - x1, 2));
            return distance <= tolerance;
        }

        private void UpdateGraphCanvas()
        {
            GraphCanvas.Children.Clear();
            foreach (var node in graph.Nodes)
            {
                DrawNode(node);
            }
            foreach (var edge in graph.Edges)
            {
                DrawEdge(edge);
            }
        }

        private async void BFSButton_Click(object sender, RoutedEventArgs e)
        {
            if (graph.Nodes.Count == 0)
            {
                MessageBox.Show("Граф пуст. Добавьте узлы перед запуском обхода.");
                return;
            }

            LogTextBox.Clear();
            await BreadthFirstSearchVisual(graph.Nodes[0]);
        }

        private async void DFSButton_Click(object sender, RoutedEventArgs e)
        {
            if (graph.Nodes.Count == 0)
            {
                MessageBox.Show("Граф пуст. Добавьте узлы перед запуском обхода.");
                return;
            }

            LogTextBox.Clear();
            await DepthFirstSearchVisual(graph.Nodes[0]);
        }

        private async Task BreadthFirstSearchVisual(Node startNode)
        {
            foreach (var node in graph.Nodes)
            {
                HighlightNode(node, Brushes.Blue);
            }

            var visited = new HashSet<Node>();
            var queue = new Queue<Node>();
            queue.Enqueue(startNode);
            LogTextBox.AppendText($"Начинаем обход в ширину с узла {startNode.Name}\n");

            while (queue.Count > 0)
            {
                var currentNode = queue.Dequeue();

                if (!visited.Contains(currentNode))
                {
                    visited.Add(currentNode);
                    LogTextBox.AppendText($"Посещаем узел {currentNode.Name}\n");

                    // Визуализируем посещение узла
                    HighlightNode(currentNode, Brushes.Red); // Красный цвет для посещенной вершины
                    await Task.Delay(5001 - _speed); // Задержка для визуализации

                    // Добавляем соседние узлы в очередь
                    foreach (var edge in graph.Edges.Where(e => e.Start == currentNode))
                    {
                        if (!visited.Contains(edge.End))
                        {
                            queue.Enqueue(edge.End);
                            LogTextBox.AppendText($"Добавляем узел {edge.End.Name} в очередь\n");

                            // Визуализируем добавление узла в очередь
                            HighlightNode(edge.End, Brushes.Green); // Зеленый цвет для добавленного узла
                            await Task.Delay(5001 - _speed); // Задержка для визуализации
                        }
                    }
                }
            }

            LogTextBox.AppendText("Обход в ширину завершен.\n");
        }

        private async Task DepthFirstSearchVisual(Node startNode)
        {
            foreach (var node in graph.Nodes)
            {
                HighlightNode(node, Brushes.Blue);
            }

            var visited = new HashSet<Node>();
            var stack = new Stack<Node>();
            stack.Push(startNode);
            LogTextBox.AppendText($"Начинаем обход в глубину с узла {startNode.Name}\n");

            while (stack.Count > 0)
            {
                var currentNode = stack.Pop();

                if (!visited.Contains(currentNode))
                {
                    visited.Add(currentNode);
                    LogTextBox.AppendText($"Посещаем узел {currentNode.Name}\n");

                    // Визуализируем посещение узла
                    HighlightNode(currentNode, Brushes.Red); // Красный цвет для посещенной вершины
                    await Task.Delay(5001 - _speed); // Задержка для визуализации

                    // Добавляем соседние узлы в стек
                    foreach (var edge in graph.Edges.Where(e => e.Start == currentNode))
                    {
                        if (!visited.Contains(edge.End))
                        {
                            stack.Push(edge.End);
                            LogTextBox.AppendText($"Добавляем узел {edge.End.Name} в стек\n");

                            // Визуализируем добавление узла в стек
                            HighlightNode(edge.End, Brushes.Green); // Зеленый цвет для добавленного узла
                            await Task.Delay(5001 - _speed); // Задержка для визуализации
                        }
                    }
                }
            }

            LogTextBox.AppendText("Обход в глубину завершен.\n");
        }

        private void HighlightNode(Node node, Brush color)
        {
            foreach (var child in GraphCanvas.Children)
            {
                if (child is Ellipse ellipse && ellipse.Tag is Node n && n == node)
                {
                    ellipse.Fill = color;
                }
            }
        }

        private void DelaySlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            _speed = (int)e.NewValue;
            DelayLabel.Content = $"Скорость: {_speed}";
        }

    }

    public class Graph
    {
        public List<Node> Nodes { get; private set; } = new List<Node>();
        public List<Edge> Edges { get; private set; } = new List<Edge>();

        public Node AddNode(string name, Point position)
        {
            Node newNode = new Node { Name = name, Position = position };
            Nodes.Add(newNode);
            return newNode;
        }

        public Edge AddEdge(Node start, Node end, int weight)
        {
            Edge newEdge = new Edge(start, end, weight);
            Edges.Add(newEdge);
            return newEdge;
        }

        public void RemoveNode(Node node)
        {
            Edges.RemoveAll(edge => edge.Start == node || edge.End == node);
            Nodes.Remove(node);
        }

        public void RemoveEdge(Edge edge)
        {
            Edges.Remove(edge);
        }

        public void Clear()
        {
            Nodes.Clear();
            Edges.Clear();
        }

        public void SaveToFile(string filePath)
        {
            using (StreamWriter writer = new StreamWriter(filePath))
            {
                // Сохраняем имена узлов
                writer.WriteLine(string.Join(";", Nodes.Select(n => n.Name)));

                int[,] adjacencyMatrix = new int[Nodes.Count, Nodes.Count];

                // Заполняем матрицу смежности
                foreach (var edge in Edges)
                {
                    int startIndex = Nodes.IndexOf(edge.Start);
                    int endIndex = Nodes.IndexOf(edge.End);
                    adjacencyMatrix[startIndex, endIndex] = edge.Weight;
                    adjacencyMatrix[endIndex, startIndex] = edge.Weight; // Если неориентированный граф
                }

                // Сохраняем матрицу смежности
                for (int i = 0; i < adjacencyMatrix.GetLength(0); i++)
                {
                    var row = new List<string>();
                    for (int j = 0; j < adjacencyMatrix.GetLength(1); j++)
                    {
                        row.Add(adjacencyMatrix[i, j].ToString());
                    }
                    writer.WriteLine(string.Join(";", row)); // Используем точку с запятой
                }
            }
        }


        public void LoadFromFile(string filePath)
        {
            using (StreamReader reader = new StreamReader(filePath))
            {
                string line = reader.ReadLine();
                if (line != null)
                {
                    var names = line.Split(';'); // Используем точку с запятой
                    int nodeCount = names.Length;

                    // Распределяем вершины по кругу
                    double radius = 300; // Радиус круга
                    for (int i = 0; i < nodeCount; i++)
                    {
                        double angle = 2 * Math.PI * i / nodeCount; // Угол в радианах
                        double x = 600 + radius * Math.Cos(angle); // Центр по X (300) + радиус * косинус угла
                        double y = 300 + radius * Math.Sin(angle); // Центр по Y (300) + радиус * синус угла
                        AddNode(names[i].Trim(), new Point(x, y)); // Добавляем вершину с рассчитанными координатами
                    }
                }

                int[,] adjacencyMatrix = new int[Nodes.Count, Nodes.Count];
                int rowIndex = 0;
                while ((line = reader.ReadLine()) != null)
                {
                    var parts = line.Split(';').Select(int.Parse).ToArray(); // Используем точку с запятой
                    for (int i = 0; i < parts.Length; i++)
                    {
                        adjacencyMatrix[rowIndex, i] = parts[i];
                    }
                    rowIndex++;
                }

                // Восстанавливаем ребра из матрицы смежности
                for (int i = 0; i < Nodes.Count; i++)
                {
                    for (int j = 0; j < Nodes.Count; j++)
                    {
                        if (adjacencyMatrix[i, j] > 0)
                        {
                            AddEdge(Nodes[i], Nodes[j], adjacencyMatrix[i, j]);
                        }
                    }
                }
            }
        }


    }

    public class Node
    {
        public string Name { get; set; }
        public Point Position { get; set; }
        public bool IsSelected { get; private set; }

        public void SetSelected(bool isSelected)
        {
            IsSelected = isSelected;
        }
    }

    public class Edge
    {
        public Node Start { get; }
        public Node End { get; }
        public int Weight { get; set; }

        public Edge(Node start, Node end, int weight)
        {
            Start = start;
            End = end;
            Weight = weight;
        }
    }
}

