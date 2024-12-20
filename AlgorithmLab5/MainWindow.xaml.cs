using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Xml.Linq;
using Label = System.Windows.Controls.Label;

namespace AlgorithmLab5
{
    public partial class MainWindow : Window
    {
        private Graph graph = new Graph();
        private Node selectedNode;
        private Edge hoveredEdge;
        private Node hoveredNode; // Добавляем переменную для отслеживания наведения на вершину
        private Node firstSelectedNode;
        private Node secondSelectedNode;
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
            else if (e.Key == Key.K)
            {
                if (hoveredNode != null)
                {
                    if (firstSelectedNode == null)
                    {
                        LogTextBox.Clear();
                        firstSelectedNode = hoveredNode;
                        HighlightNode(firstSelectedNode, Brushes.Yellow); // Выбираем первый узел
                        LogTextBox.AppendText($"Выбран первый узел: {firstSelectedNode.Name}\n");
                    }
                    else if (secondSelectedNode == null && hoveredNode != firstSelectedNode)
                    {
                        secondSelectedNode = hoveredNode;
                        HighlightNode(secondSelectedNode, Brushes.Yellow); // Выбираем второй узел
                        LogTextBox.AppendText($"Выбран второй узел: {secondSelectedNode.Name}\n");
                        // Запускаем алгоритм после выбора двух узлов
                        ShortestPathButton_Click(null, null);
                    }
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

            // лейбл над нодой, указывающий расстояние до нее в алгоритме Дейкстры
            SolidColorBrush mySolidColorBrush = new SolidColorBrush(Colors.Gray); // Цвет
            mySolidColorBrush.Opacity = 0.0;                                     // Прозрачность. 0.0 - прозрачный фон
            Label DistanceLabel = new Label
            {
                Width = 40,
                Height = 30,
                Content = "",
                Tag = node,

                Background = mySolidColorBrush,
                BorderThickness = new Thickness(2),
                VerticalAlignment = VerticalAlignment.Top,
                HorizontalAlignment = HorizontalAlignment.Center
            };

            Canvas.SetLeft(DistanceLabel, node.Position.X + 40);
            Canvas.SetTop(DistanceLabel, node.Position.Y - 15);
            GraphCanvas.Children.Add(DistanceLabel);

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
                Tag = edge
            };
            GraphCanvas.Children.Insert(0, line);
            edge.Line = line; // Store the line reference in the edge

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
            edge.WeightTextBox = weightTextBox; // Store the TextBox reference in the edge

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

            foreach (var edge in graph.Edges)
            {
                HighlightEdge(edge, Brushes.Black);
            }

            foreach (var node in graph.Nodes)
            {
                HighlightNode(node, Brushes.Blue);
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

            foreach (var edge in graph.Edges)
            {
                HighlightEdge(edge, Brushes.Black);
            }

            foreach (var node in graph.Nodes)
            {
                HighlightNode(node, Brushes.Blue);
            }

            LogTextBox.Clear();
            await DepthFirstSearchVisual(graph.Nodes[0]);
        }

        private async Task BreadthFirstSearchVisual(Node startNode)
        {
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
                    foreach (var edge in graph.Edges.Where(e => e.Start == currentNode || e.End == currentNode))
                    {
                        if (!visited.Contains(edge.End))
                        {
                            queue.Enqueue(edge.End);
                            LogTextBox.AppendText($"Добавляем узел {edge.End.Name} в очередь\n");

                            // Визуализируем добавление узла в очередь
                            HighlightNode(edge.End, Brushes.Green); // Зеленый цвет для добавленного узла
                            await Task.Delay(5001 - _speed);
                        }
                    }
                }
            }

            LogTextBox.AppendText("Обход в ширину завершен.\n");
        }


        private async Task DepthFirstSearchVisual(Node startNode)
        {
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
                    foreach (var edge in graph.Edges.Where(e => e.Start == currentNode || e.End == currentNode))
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


        private async void MaxFlowButton_Click(object sender, RoutedEventArgs e)
        {
            if (graph.Nodes.Count <= 1)
            {
                MessageBox.Show("Граф пуст. Добавьте узлы перед запуском алгоритма.");
                return;
            }

            foreach (var node in graph.Nodes)
            {
                HighlightNode(node, Brushes.Blue);
            }

            foreach (var edge in graph.Edges)
            {
                HighlightEdge(edge, Brushes.Black);
            }

            LogTextBox.Clear();
            await MaxFlow(graph.Nodes[0], graph.Nodes.Last());
        }

        private async Task MaxFlow(Node source, Node sink)
        {
            int n = graph.Nodes.Count;
            var capacityMatrix = new int[n, n];
            var residualMatrix = new int[n, n];
            var parent = new int[n];
            var labels = new List<Label>();

            // Initialize capacity and residual matrices
            foreach (var edge in graph.Edges)
            {
                int startIndex = graph.Nodes.IndexOf(edge.Start);
                int endIndex = graph.Nodes.IndexOf(edge.End);
                capacityMatrix[startIndex, endIndex] = edge.Weight;
                residualMatrix[startIndex, endIndex] = edge.Weight; // Initial residual capacity
            }

            int maxFlow = 0;

            while (true)
            {
                // Perform BFS to find an augmenting path
                bool foundPath = false;
                var visited = new bool[n];
                Queue<int> queue = new Queue<int>();
                queue.Enqueue(graph.Nodes.IndexOf(source));
                visited[graph.Nodes.IndexOf(source)] = true;
                parent[graph.Nodes.IndexOf(source)] = -1;

                while (queue.Count > 0)
                {
                    int u = queue.Dequeue();

                    for (int v = 0; v < n; v++)
                    {
                        if (!visited[v] && residualMatrix[u, v] > 0) // If not visited and there's residual capacity
                        {
                            queue.Enqueue(v);
                            parent[v] = u;
                            visited[v] = true;

                            if (v == graph.Nodes.IndexOf(sink)) // If we reached the sink
                            {
                                foundPath = true;
                                break;
                            }
                        }
                    }

                    if (foundPath)
                        break;
                }

                if (!foundPath)
                    break; // No more augmenting paths

                // Find the maximum flow through the path found
                int pathFlow = int.MaxValue;
                for (int v = graph.Nodes.IndexOf(sink); v != graph.Nodes.IndexOf(source); v = parent[v])
                {
                    int u = parent[v];
                    pathFlow = Math.Min(pathFlow, residualMatrix[u, v]);
                }

                // Update residual capacities of the edges and reverse edges along the path
                var logEntries = new List<string>(); // Список для хранения логов
                for (int v = graph.Nodes.IndexOf(sink); v != graph.Nodes.IndexOf(source); v = parent[v])
                {
                    int u = parent[v];
                    residualMatrix[u, v] -= pathFlow; // Decrease residual capacity
                    residualMatrix[v, u] += pathFlow; // Increase reverse capacity

                    // Update edge weights for visualization
                    var edge = graph.Edges.First(e => e.Start == graph.Nodes[u] && e.End == graph.Nodes[v]);
                    int oldWeight = edge.Weight; // Сохраняем старый вес для логирования
                    edge.Weight -= pathFlow; // Update the edge weight

                    double midX = (edge.Start.Position.X + edge.End.Position.X) / 2 + 30; // Положение X
                    double midY = (edge.Start.Position.Y + edge.End.Position.Y) / 2 + 30; // Положение Y

                    Label weightLabel = new Label
                    {
                        Content = $"{oldWeight}/{pathFlow}", // Текст метки
                        Width = 40,
                        Height = 30,
                        Background = Brushes.Gray,
                        Foreground = Brushes.Black,
                        HorizontalAlignment = HorizontalAlignment.Left,
                        VerticalAlignment = VerticalAlignment.Top
                    };

                    Canvas.SetLeft(weightLabel, midX);
                    Canvas.SetTop(weightLabel, midY);
                    GraphCanvas.Children.Add(weightLabel);
                    labels.Add(weightLabel);

                    edge.WeightTextBox.Visibility = Visibility.Collapsed;
                    edge.Line.Stroke = Brushes.Red;

                    // Формируем строку логирования
                    logEntries.Add($"Обновляем ребро {edge.Start.Name} -> {edge.End.Name} ({oldWeight}) с новым остаточным потоком {oldWeight} - {pathFlow} = {edge.Weight}");
                }

                // Выводим логирование в обратном порядке
                foreach (var logEntry in logEntries.AsEnumerable().Reverse())
                {
                    LogTextBox.AppendText(logEntry + "\n");
                    await Task.Delay(5001 - _speed); // Delay for visualization
                }

                LogTextBox.AppendText($"Текущий максимальный поток: {maxFlow} + {pathFlow} = {maxFlow + pathFlow}\n");
                maxFlow += pathFlow;

                foreach (var edge in graph.Edges)
                {
                    edge.WeightTextBox.Visibility = Visibility.Visible;
                    edge.Line.Stroke = Brushes.Black;
                }

                foreach(var lable in labels)
                {
                    GraphCanvas.Children.Remove(lable);
                }
            }

            // Log the final results
            LogTextBox.AppendText($"Максимальный поток: {maxFlow}\n");
            await Task.Delay(5001 - _speed);

            // Restore original edge weights if needed
            foreach (var edge in graph.Edges)
            {
                edge.Weight = capacityMatrix[graph.Nodes.IndexOf(edge.Start), graph.Nodes.IndexOf(edge.End)];
                edge.WeightTextBox.Text = $"{edge.Weight}"; // Update TextBox
                edge.Line.Stroke = Brushes.Black;
            }
        }


        private async void MSTButton_Click(object sender, RoutedEventArgs e)
        {
            if (graph.Nodes.Count == 0)
            {
                MessageBox.Show("Граф пуст. Добавьте узлы перед запуском алгоритма.");
                return;
            }

            foreach (var node in graph.Nodes)
            {
                HighlightNode(node, Brushes.Blue);
            }

            foreach (var edge in graph.Edges)
            {
                HighlightEdge(edge, Brushes.Black);
            }

            LogTextBox.Clear();
            await PrimMST();
        }

        private async Task PrimMST()
        {
            // Проверяем, является ли граф связным
            if (graph.Edges.Count < graph.Nodes.Count - 1)
            {
                LogTextBox.AppendText("Граф несвязный! Невозможно построить остовное дерево.\n");
                return;
            }

            var mstEdges = new List<Edge>();
            var visited = new HashSet<Node>();
            var priorityQueue = new SortedSet<(int weight, Node start, Node end)>(Comparer<(int weight, Node start, Node end)>.Create((x, y) =>
            {
                int weightComparison = x.weight.CompareTo(y.weight);
                if (weightComparison != 0) return weightComparison;

                // Если веса равны, сравниваем по меньшей вершине
                int startComparison = x.start.Name.CompareTo(y.start.Name);
                if (startComparison != 0) return startComparison;

                return x.end.Name.CompareTo(y.end.Name);
            }));

            // Начинаем с первого узла
            var startNode = graph.Nodes[0];
            visited.Add(startNode);

            // Добавляем все рёбра, исходящие из начального узла
            foreach (var edge in graph.Edges.Where(e => e.Start == startNode || e.End == startNode))
            {
                priorityQueue.Add((edge.Weight, edge.Start, edge.End));
            }

            LogTextBox.AppendText($"Начинаем построение минимального остовного дерева с узла {startNode.Name}\n");

            while (priorityQueue.Count > 0)
            {
                var (weight, start, end) = priorityQueue.Min;
                priorityQueue.Remove(priorityQueue.Min);

                // Проверяем, добавляем ли мы новое ребро
                if (visited.Contains(start) && visited.Contains(end))
                    continue;

                // Добавляем ребро в остовное дерево
                Edge edgeToAdd = graph.Edges.First(e => (e.Start == start && e.End == end) || (e.Start == end && e.End == start));
                mstEdges.Add(edgeToAdd);
                LogTextBox.AppendText($"Добавляем ребро {edgeToAdd.Start.Name} - {edgeToAdd.End.Name} с весом {edgeToAdd.Weight}\n");

                edgeToAdd.Line.Stroke = Brushes.Red;
                await Task.Delay(5001 - _speed); // Задержка для визуализации

                // Добавляем соседние узлы в очередь
                Node newNode = visited.Contains(edgeToAdd.Start) ? edgeToAdd.End : edgeToAdd.Start;
                visited.Add(newNode);

                foreach (var nextEdge in graph.Edges.Where(e => e.Start == newNode || e.End == newNode))
                {
                    if (!visited.Contains(nextEdge.Start) || !visited.Contains(nextEdge.End))
                    {
                        priorityQueue.Add((nextEdge.Weight, nextEdge.Start, nextEdge.End));
                    }
                }
            }

            LogTextBox.AppendText($"Минимальное остовное дерево построено с {mstEdges.Count} рёбрами).\n");

            // Создаем граф для остовного дерева
            var uniqueNodes = mstEdges.SelectMany(e => new[] { e.Start, e.End }).Distinct().ToList();
            Graph mstGraph = Graph.GetGraph(mstEdges, uniqueNodes);

            // Сохраняем остовное дерево в файл
            SaveMSTToFile(mstGraph);
        }




        private async void ShortestPathButton_Click(object sender, RoutedEventArgs e)
        {
            if (firstSelectedNode == null || secondSelectedNode == null)
            {
                MessageBox.Show("Пожалуйста, выберите две вершины.");
                return;
            }

            foreach (var node in graph.Nodes)
            {
                HighlightNode(node, Brushes.Blue);
            }

            foreach (var edge in graph.Edges)
            {
                HighlightEdge(edge, Brushes.Black);
            }

            await Dijkstra(firstSelectedNode, secondSelectedNode);
        }

        private async Task Dijkstra(Node startNode, Node endNode)
        {
            var distances = new Dictionary<Node, int>();
            var previousNodes = new Dictionary<Node, Node>();
            var previousEdges = new List<Edge>();
            var unvisitedNodes = new HashSet<Node>(graph.Nodes);

            // Инициализация расстояний
            foreach (var node in graph.Nodes)
            {
                distances[node] = int.MaxValue;
                previousNodes[node] = null;
                Update_Distance(node, "卐"); // ∞ 
            }

            distances[startNode] = 0;
            Update_Distance(startNode, 0);

            LogTextBox.AppendText($"Начальная вершина: {startNode.Name}.\n");

            while (unvisitedNodes.Count > 0)
            {
                LogTextBox.AppendText("⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯⋯\n");
                Node currentNode = unvisitedNodes.OrderBy(n => distances[n]).First();
                LogTextBox.AppendText($"Посещаем вершину {currentNode.Name} с текущим расстоянием {distances[currentNode]}.\n"); // {distances[currentNode] - int.MinValue + 1}
                if (currentNode == endNode)
                {
                    LogTextBox.AppendText("Конечная вершина достигнута.\n");
                    Update_Distance(currentNode, distances[currentNode]);
                    break;
                }

                unvisitedNodes.Remove(currentNode);

                foreach (var edge in graph.Edges.Where(e => e.Start == currentNode || e.End == currentNode))
                {
                    HighlightEdge(edge, Brushes.Red, 5);

                    Node neighbor = (edge.Start == currentNode) ? edge.End : edge.Start;
                    if (unvisitedNodes.Contains(neighbor))
                    {
                        int newDist = distances[currentNode] + edge.Weight;
                        LogTextBox.AppendText("⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫⬫\n");
                        LogTextBox.AppendText($"Проверяем соседнюю вершину {neighbor.Name} через ребро с весом {edge.Weight}.\n");

                        if (newDist < distances[neighbor])
                        {
                            LogTextBox.AppendText($"> Обновляем расстояние до вершины {neighbor.Name}: {newDist}.\n");
                            distances[neighbor] = newDist;
                            previousNodes[neighbor] = currentNode;
                            previousEdges.Add(edge);
                            Update_Distance(neighbor, newDist);

                            // Логирование пути
                            string path = $"{startNode.Name} -> {neighbor.Name}";
                            LogTextBox.AppendText($"Путь: {path}.\n");

                            // Визуализация
                            HighlightNode(neighbor, Brushes.Green);
                            await Task.Delay(5001 - _speed); // Задержка для визуализации
                        }
                        else
                        {
                            LogTextBox.AppendText($"> Расстояние до вершины {neighbor.Name} не обновляется, текущее значение {distances[neighbor]} меньше нового { newDist }.\n");
                        }
                    }
                    HighlightEdge(edge, Brushes.Black, 1);
                }
            }

            // Восстановление пути
            var pathStack = new Stack<Node>();
            var pathStackClone = new Stack<Node>();
            var pathEdgesStack = new Stack<Edge>();
            for (Node at = endNode; at != null; at = previousNodes[at])
            {
                pathStack.Push(at);
                pathStackClone.Push(at);
            }

            pathEdgesStack = GetPathEdges(pathStackClone);
            foreach (Edge edge in pathEdgesStack) 
            {
                HighlightEdge(edge, Brushes.Red, 5);
            }
            LogTextBox.AppendText("Кратчайший путь: ");
            while (pathStack.Count > 0)
            {
                Node nodeInPath = pathStack.Pop();
                LogTextBox.AppendText($"{nodeInPath.Name}");
                if (pathStack.Count > 0)
                {
                    LogTextBox.AppendText(" -> ");
                }

                HighlightNode(nodeInPath, Brushes.Red); // Подсветка кратчайшего пути
                await Task.Delay(5001 - _speed); // Задержка для визуализации
            }

            

            LogTextBox.AppendText("\n");

            // Сброс выбранных узлов
            firstSelectedNode = null;
            secondSelectedNode = null;
        }

        private Stack<Edge> GetPathEdges(Stack<Node> pathStack)
        {
            var pathEdgesStack = new Stack<Edge>();
            int length = pathStack.Count;
            if (length > 1)
            {
                Node node = pathStack.Pop();
                for (int i = 0; i < length - 1; i++)
                {
                    Node nextNode = pathStack.Pop();

                    foreach (Edge edge in graph.Edges)
                    {
                        if (edge.Start == node && edge.End == nextNode)
                        {
                            pathEdgesStack.Push(edge);
                        }
                    }
                    node = nextNode;
                }
            }
            return pathEdgesStack;
        }

        private Stack<Edge> GetPathEdges(Dictionary<Node, Node> path)
        {

            var pathEdgesStack = new Stack<Edge>();

            foreach (KeyValuePair<Node, Node> nodes in path)
            {
                foreach (Edge edge in graph.Edges)
                {
                    if (edge.Start == nodes.Key && edge.End == nodes.Value) 
                    {
                        pathEdgesStack.Push(edge);
                    }
                }
            }
            return pathEdgesStack;
        }

        private void HighlightEdge(Edge edge, Brush color)
        {
            edge.Line.Stroke = color;
        }
        private void HighlightEdge(Edge edge, Brush color, int thickness)
        {
            edge.Line.Stroke = color;
            edge.Line.StrokeThickness = thickness;
        }

        private void SaveMSTToFile(Graph mstGraph)
        {
            SaveFileDialog saveFileDialog = new SaveFileDialog
            {
                Filter = "CSV files (*.csv)|*.csv|All files (*.*)|*.*",
                Title = "Save Minimum Spanning Tree"
            };

            if (saveFileDialog.ShowDialog() == true)
            {
                mstGraph.SaveToFile(saveFileDialog.FileName);
            }
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

        private void Update_Distance(Node node, int distance)
        {
            foreach (var child in GraphCanvas.Children)
            {
                if (child is Label label && label.Tag is Node n && n == node)
                {
                    label.Content = distance;
                }
            }
        }

        private void Update_Distance(Node node, string distance) // Джинерики для слабаков
        {
            foreach (var child in GraphCanvas.Children)
            {
                if (child is Label label && label.Tag is Node n && n == node)
                {
                    label.Content = distance;
                }
            }
        }


    }

    public class Graph
    {
        public List<Node> Nodes { get; private set; } = new List<Node>();
        public List<Edge> Edges { get; private set; } = new List<Edge>();

        public static Graph GetGraph(List<Edge> edges, List<Node> nodes)
        {
            return new Graph { Edges = edges, Nodes = nodes };
        }

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
        //public int distance { get; set; }
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
        public TextBox WeightTextBox { get; set; } // Add this line to hold the TextBox reference
        public Line Line { get; set; } // Add this line to hold the Line reference

        public Edge(Node start, Node end, int weight)
        {
            Start = start;
            End = end;
            Weight = weight;
        }
    }
}

