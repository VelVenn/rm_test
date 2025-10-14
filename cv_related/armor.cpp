#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

struct Rect {
    bool color;
    int width, height, id;
    pair<int, int> point;

    Rect(int id = 1, bool color = 0, int width = 0, int height = 0, pair<int, int> point = { 0, 0 })
        : color(color)
        , width(width)
        , height(height)
        , id(id)
        , point(point)
    {
    }
};

class Armor {
private:
    Rect rect;

public:
    Armor(Rect rect = Rect())
        : rect(rect)
    {
    }

    void setRect(Rect rect)
    {
        this->rect = rect;
    }

    pair<int, int> getCenter() const // 获得中心点位置
    {
        return { rect.point.first + rect.width / 2, rect.point.second + rect.height / 2 };
    }

    double diagonal() const // 获得对角线长度
    {
        return hypot(rect.width, rect.height);
    }

    string getDiagonalWithPrec(int precision = 2) const // 获得对角线长度并保留precision位小数
    {
        stringstream stream;
        stream << fixed << setprecision(precision) << diagonal();
        return stream.str();
    }

    string getColor() const // 获得颜色
    {
        return rect.color ? "红" : "蓝";
    }

    vector<pair<int, int>> getVertices() const // 获得四个顶点坐标
    {
        vector<pair<int, int>> vertices;
        vertices.push_back(rect.point);
        vertices.push_back({ rect.point.first + rect.width, rect.point.second });
        vertices.push_back({ rect.point.first + rect.width, rect.point.second + rect.height });
        vertices.push_back({ rect.point.first, rect.point.second + rect.height });
        return vertices;
    }
};

int main()
{
    int id, color, width, height, x, y;
    cin >> id >> color >> x >> y >> width >> height;
    Armor armor(Rect(id, color, width, height, { x, y }));

    cout << "ID: " << id << " 颜色: " << armor.getColor() << endl;
    cout << "(" << armor.getCenter().first << ", " << armor.getCenter().second << ") ";
    cout << "长度: " << armor.getDiagonalWithPrec() << endl;

    vector<pair<int, int>> vertices = armor.getVertices();
    for (auto vertex : vertices) {
        cout << "(" << vertex.first << ", " << vertex.second << ") ";
    }
}