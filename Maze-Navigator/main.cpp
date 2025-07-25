
#define _CRT_SECURE_NO_WARNINGS

#include <vgl.h>
#include <InitShader.h>
#include <iostream>
#include <vector>
#include <queue>
#include "MyCube.h"
#include "MyUtil.h"

#include <vec.h>
#include <mat.h>

#define MAZE_FILE	"maze.txt"

using namespace std;

MyCube cube;
GLuint program;

mat4 g_Mat = mat4(1.0f);
GLuint uMat;
GLuint uColor;

float wWidth = 1000;
float wHeight = 500;

vec3 cameraPos = vec3(0, 0, 0);
vec3 viewDirection = vec3(0, 0, -1);
vec3 goalPos = vec3(0, 0, 0);

int MazeSize;
char maze[255][255] = { 0 };

float cameraSpeed = 0.1;

float g_time = 0;

float cameraYaw = -90.0f; // ȸ�� ����

pair <int, int> goalIndex;
pair <int, int> startIndex;

typedef struct Node {
	pair<int, int> idx;
	float f, g, h;
	pair<int, int> parent;

	// �켱������ �����ϱ� ���� �� ������
	bool operator<(const Node& other) const {
		return f > other.f; // f���� ���� ���� �켱������ ����
	}
} Node;

priority_queue<Node> openNode;
vector<Node> closeNode;
vector<pair<int, int>> path;

enum CameraState {
	STOPPED,      // ���� ����
	MOVING,       // �̵� �� ����
	PAUSED        // �Ͻ� ���� ����
};

CameraState cameraState = STOPPED;
bool isFollowingPath = false;   // ī�޶� ��θ� ���� �̵� ������ ����
int currentPathIndex = 0;
bool spacePressed = false;

typedef struct PathPos {
	vec3 pos;
	vec3 dir;
	float ang;
}PathPos;

vector<PathPos> pathPos;

pair<int, int> getIndexFromPosition(vec3 position)
{
	pair<int, int> idx;
	idx.first = round(position.x + MazeSize / 2.0f - 0.5f);
	idx.second = round(position.z + MazeSize / 2.0f - 0.5f);

	return idx;
}

bool isCollision(vec3 position) {
	pair<int, int> idx = getIndexFromPosition(position);
	return maze[idx.first][idx.second] == '*';
}

inline vec3 getPositionFromIndex(int i, int j)
{
	float unit = 1;
	vec3 leftTopPosition = vec3(-MazeSize / 2.0 + unit / 2, 0, -MazeSize / 2.0 + unit / 2);
	vec3 xDir = vec3(1, 0, 0);
	vec3 zDir = vec3(0, 0, 1);
	return leftTopPosition + i * xDir + j * zDir;
}

void LoadMaze()
{
	FILE* file = fopen(MAZE_FILE, "r");
	char buf[255];
	fgets(buf, 255, file);
	sscanf(buf, "%d", &MazeSize);
	for (int j = 0; j < MazeSize; j++)
	{
		fgets(buf, 255, file);
		for (int i = 0; i < MazeSize; i++)
		{
			maze[i][j] = buf[i];
			if (maze[i][j] == 'C')				// Setup Camera Position
				cameraPos = getPositionFromIndex(i, j);
			if (maze[i][j] == 'G') {			// Setup Goal Position
				goalPos = getPositionFromIndex(i, j);
				goalIndex.first = j;
				goalIndex.second = i;
			}
		}
	}
	fclose(file);
}

int Manhattan(pair<int, int> idx)
{
	return abs(goalIndex.first - idx.first) + abs(goalIndex.second - idx.second);
}

bool checkCloseNode(Node p) {
	pair<int, int> idx1 = p.idx;

	for (int i = 0; i < closeNode.size(); i++) {
		pair<int, int> idx2 = closeNode[i].idx;
		if (idx1 == idx2)
			return true;
	}
	return false;
}

void setNode(Node curNode) {
	pair<int, int> curIdx = curNode.idx;
	int dirX[4] = { 1, -1, 0, 0 };
	int dirY[4] = { 0, 0, 1, -1 };

	for (int i = 0; i < 4; i++) {
		pair<int, int> idx = { curIdx.first + dirX[i], curIdx.second + dirY[i] };

		// ��ֹ� ó�� �Ǵ� �̹� closeNode�� �ִ� ��� ����
		bool isInCloseNode = false;
		for (const Node& n : closeNode) {
			if (n.idx == idx) {
				isInCloseNode = true;
				break;
			}
		}

		if (maze[idx.first][idx.second] == '*' || isInCloseNode) {
			continue;
		}

		Node p;
		p.idx = idx;
		p.h = Manhattan(p.idx);
		p.g = curNode.g + 1;
		p.f = p.g + p.h;
		p.parent = curNode.idx;

		// openNode�� �߰�
		openNode.push(p);
	}
}

void Astar() {
	while (!openNode.empty()) openNode.pop(); // �ʱ�ȭ
	closeNode.clear();

	// ���� ��� ����
	startIndex = getIndexFromPosition(cameraPos);
	Node startNode;
	startNode.idx = startIndex;
	startNode.h = Manhattan(startIndex);
	startNode.g = 0;
	startNode.f = startNode.h + startNode.g;

	openNode.push(startNode);

	Node curNode;

	while (!openNode.empty()) {
		// openNode���� ���� ����� ���� ��� ����
		curNode = openNode.top();
		openNode.pop();

		// closeNode�� �߰�
		closeNode.push_back(curNode);

		// ��ǥ ��忡 �����ϸ� ����
		if (curNode.idx == goalIndex) {
			break;
		}

		// ���� ��� Ž��
		setNode(curNode);
	}

	// ��� ����
	path.clear();
	while (curNode.idx != startIndex) {
		path.push_back(curNode.idx);
		Node* parent = nullptr;
		for (Node& n : closeNode) {
			if (n.idx == curNode.parent) {
				parent = &n;
				break;
			}
		}
		if (parent)
			curNode = *parent;
		else {
			break; // ��� ���� �ߴ� (���� ����)
		}
	}
	path.push_back(startIndex); // ������ �߰�
	reverse(path.begin(), path.end()); // ��� ����
}
float smooth(float t) {
	return t * t * (4 - 3 * t);
}
void navigatePath() {
	pathPos.clear();
	for (int i = 0; i < path.size() - 1; i++) {
		vec3 currentPos = getPositionFromIndex(path[i].first, path[i].second);
		vec3 nextPos = getPositionFromIndex(path[i + 1].first, path[i + 1].second);

		// ���� ����� ���� ���� ���
		vec3 currentDir = vec3(0, 0, -1);  // �ʱ� ����
		if (i > 0) {
			// ���� ��ġ������ ������ ���
			vec3 prevPos = getPositionFromIndex(path[i - 1].first, path[i - 1].second);
			currentDir = normalize(currentPos - prevPos);
		}
		vec3 nextDir = normalize(nextPos - currentPos);

		// ���� ������ ��ǥ ���� ���
		float currentAngle = atan2(currentDir.z, currentDir.x) * 180.0f / 3.141592;
		float targetAngle = atan2(nextDir.z, nextDir.x) * 180.0f / 3.141592;

		float angleDiff = targetAngle - currentAngle;
		if (angleDiff > 180) angleDiff -= 360;
		if (angleDiff < -180) angleDiff += 360;

		float moveProgress = 0.0f;
		while (moveProgress <= 1.0f) {
			PathPos p;
			// ��ġ ����
			p.pos = currentPos * (1.0f - moveProgress) + nextPos * moveProgress;

			// ���� ����
			float rotationProgress = smooth(moveProgress);  // �ε巯�� ������ ���� �Լ�
			float interpolatedAngle = currentAngle + angleDiff * rotationProgress;
			p.ang = interpolatedAngle;

			// ���� ���� ���
			float radians = interpolatedAngle * 3.141592f / 180.0f;
			p.dir = vec3(cos(radians), 0, sin(radians));

			pathPos.push_back(p);
			moveProgress += 0.05;
		}

	}
}

void DrawPath() {
	if (path.empty())
		return;

	vec3 color = vec3(1, 0, 0);
	float pathWidth = 0.1f;

	for (size_t i = 0; i < path.size() - 1; i++) {
		// �������� ������ ��ǥ�� ���
		vec3 startPos = getPositionFromIndex(path[i].first, path[i].second);
		vec3 endPos = getPositionFromIndex(path[i + 1].first, path[i + 1].second);

		// �� �� ������ �߰��� ���
		vec3 midPos = (startPos + endPos) / 2.0f;
		midPos.y = -0.4;

		// �� �� ������ �Ÿ� ���
		vec3 diff = endPos - startPos;
		float length = sqrt(dot(diff, diff));

		// ���� ��� (diff ������ ����)
		float angle = atan2(diff.z, diff.x) * 180.0f / 3.141592;

		mat4 ModelMat = Translate(midPos) * RotateY(angle) * Scale(vec3(length, 0.02f, pathWidth));

		glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
		glUniform4f(uColor, color.x, color.y, color.z, 1);
		cube.Draw(program);
	}
}

void DrawMaze()
{
	for (int j = 0; j < MazeSize; j++)
		for (int i = 0; i < MazeSize; i++)
			if (maze[i][j] == '*')
			{
				vec3 color = vec3(i / (float)MazeSize, j / (float)MazeSize, 1);
				if (isCollision(cameraPos))
					color = vec3(255, 0, 0);
				mat4 ModelMat = Translate(getPositionFromIndex(i, j));
				glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
				glUniform4f(uColor, color.x, color.y, color.z, 1);
				cube.Draw(program);
			}
}

void myInit()
{
	LoadMaze();
	cube.Init();
	program = InitShader("vshader.glsl", "fshader.glsl");

}

void DrawGrid()
{
	float n = 40;
	float w = MazeSize;
	float h = MazeSize;

	for (int i = 0; i < n; i++)
	{
		mat4 m = Translate(0, -0.5, -h / 2 + h / n * i) * Scale(w, 0.02, 0.02);
		glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * m);
		glUniform4f(uColor, 1, 1, 1, 1);
		cube.Draw(program);
	}
	for (int i = 0; i < n; i++)
	{
		mat4 m = Translate(-w / 2 + w / n * i, -0.5, 0) * Scale(0.02, 0.02, h);
		glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * m);
		glUniform4f(uColor, 1, 1, 1, 1);
		cube.Draw(program);
	}
}


void drawCamera()
{
	float cameraSize = 0.5;

	mat4 ModelMat = Translate(cameraPos) * Scale(vec3(cameraSize)) * RotateY(-cameraYaw);
	glUseProgram(program);
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
	glUniform4f(uColor, 0, 1, 0, 1);
	cube.Draw(program);

	ModelMat = Translate(cameraPos + viewDirection * cameraSize / 2) * Scale(vec3(cameraSize / 2)) * RotateY(-cameraYaw);
	glUseProgram(program);
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
	glUniform4f(uColor, 0, 1, 0, 1);
	cube.Draw(program);
}

void drawGoal()
{
	glUseProgram(program);
	float GoalSize = 0.7;

	mat4 ModelMat = Translate(goalPos) * RotateY(g_time * 3) * Scale(vec3(GoalSize));
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
	glUniform4f(uColor, 0, 0, 0, 0);
	cube.Draw(program);

	ModelMat = Translate(goalPos) * RotateY(g_time * 3 + 45) * Scale(vec3(GoalSize));
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
	glUniform4f(uColor, 0, 0, 0, 0);
	cube.Draw(program);
}


void drawScene(bool bDrawCamera = true)
{
	glUseProgram(program);
	uMat = glGetUniformLocation(program, "uMat");
	uColor = glGetUniformLocation(program, "uColor");

	DrawGrid();
	DrawMaze();
	drawGoal();
	DrawPath();

	if (bDrawCamera)
		drawCamera();
}

void display()
{
	glEnable(GL_DEPTH_TEST);

	float vWidth = wWidth / 2;
	float vHeight = wHeight;

	// LEFT SCREEN : View From Camera (Perspective Projection)
	glViewport(0, 0, vWidth, vHeight);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	float h = 4;
	float aspectRatio = vWidth / vHeight;
	float w = aspectRatio * h;
	mat4 ViewMat = myLookAt(cameraPos, cameraPos + viewDirection, vec3(0, 1, 0));
	mat4 ProjMat = myPerspective(45, aspectRatio, 0.01, 20);

	g_Mat = ProjMat * ViewMat;
	drawScene(false);							// drawing scene except the camera


	// RIGHT SCREEN : View from above (Orthographic parallel projection)
	glViewport(vWidth, 0, vWidth, vHeight);
	h = MazeSize;
	w = aspectRatio * h;
	ViewMat = myLookAt(vec3(0, 5, 0), vec3(0, 0, 0), vec3(0, 0, -1));
	ProjMat = myOrtho(-w / 2, w / 2, -h / 2, h / 2, 0, 20);

	g_Mat = ProjMat * ViewMat;
	drawScene(true);


	glutSwapBuffers();
}

void idle()
{
	g_time += 1;

	if (isFollowingPath && cameraState == MOVING) {
		if (currentPathIndex < pathPos.size()) {
			// ���� ��� ��ġ ������Ʈ
			cameraPos = pathPos[currentPathIndex].pos;
			cameraYaw = pathPos[currentPathIndex].ang;
			viewDirection = pathPos[currentPathIndex].dir;

			// ���� ��η� �̵�
			currentPathIndex++;
		}
		else {
			// ��� ���� �����ϸ� �̵� ����
			isFollowingPath = false;
			cameraState = STOPPED;
			currentPathIndex = 0; // �ʱ�ȭ
		}
	}
	else {
		// A�� D�� ī�޶� ȸ��
		if ((GetAsyncKeyState('A') & 0x8000) == 0x8000) { // Left
			cameraYaw -= 3.0f;
		}
		if ((GetAsyncKeyState('D') & 0x8000) == 0x8000) { // Right
			cameraYaw += 3.0f;
		}

		float yawRad = cameraYaw / 180.0f * 3.141592;
		viewDirection = vec3(cos(yawRad), 0, sin(yawRad));


		if ((GetAsyncKeyState('W') & 0x8000) == 0x8000) {
			vec3 newPos = cameraPos + cameraSpeed * viewDirection;
			if (!isCollision(newPos)) {
				cameraPos = newPos;
			}
		}
		if ((GetAsyncKeyState('S') & 0x8000) == 0x8000) {
			vec3 newPos = cameraPos - cameraSpeed * viewDirection;
			if (!isCollision(newPos)) {
				cameraPos = newPos;
			}
		}
	}
	if ((GetAsyncKeyState('Q') & 0x8000) == 0x8000) {
		Astar();
		navigatePath();
	}
	if ((GetAsyncKeyState(VK_SPACE) & 0x8000) == 0x8000) {
		if (!spacePressed) {  // Ű�� ó�� ������ ���� ó��
			switch (cameraState) {
			case STOPPED:
				// ó������ �̵� ����
				currentPathIndex = 0;
				isFollowingPath = true;
				cameraState = MOVING;
				break;

			case MOVING:
				// �̵� ���̸� �Ͻ� ����
				isFollowingPath = false;
				cameraState = PAUSED;
				break;

			case PAUSED:
				// �Ͻ� ���� ���¿��� �ٽ� ó������ �̵� ����
				currentPathIndex = 0;
				isFollowingPath = true;
				cameraState = MOVING;
				break;
			}
			spacePressed = true;
		}
	}
	else {
		spacePressed = false;  // Ű�� �������� �� �÷��� ����
	}
	Sleep(16);											// for vSync
	glutPostRedisplay();
}

void reshape(int wx, int wy)
{
	printf("%d %d \n", wx, wy);
	wWidth = wx;
	wHeight = wy;
	glutPostRedisplay();
}


int main(int argc, char** argv)
{
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(wWidth, wHeight);

	glutCreateWindow("Homework3 (Maze Navigator)");

	glewExperimental = true;
	glewInit();

	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION),
		glGetString(GL_SHADING_LANGUAGE_VERSION));

	myInit();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);
	glutMainLoop();

	return 0;
}