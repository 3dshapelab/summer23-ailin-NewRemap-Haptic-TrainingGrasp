#include <windows.h>

#include <gl/glew.h> // http://glew.sourceforge.net/
#include <gl/wglew.h>

#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library


#include "string.h"

#include <iostream>



class CShaderProgram
{
public:
	GLuint *UniformLocations;

protected:
	GLuint VertexShader, FragmentShader, Program;

public:
	CShaderProgram();
	~CShaderProgram();

	operator GLuint ();

	void Delete();
	bool Load(char *VertexShaderFileName, char *FragmentShaderFileName);

protected:
	GLuint LoadShader(GLenum Type, char *ShaderFileName);
	void SetDefaults();
};

inline CShaderProgram::CShaderProgram()
{
	SetDefaults();
}

inline CShaderProgram::~CShaderProgram()
{
}

inline CShaderProgram::operator GLuint ()
{
	return Program;
}

inline void CShaderProgram::Delete()
{
	delete [] UniformLocations;

	glDetachShader(Program, VertexShader);
	glDetachShader(Program, FragmentShader);

	glDeleteShader(VertexShader);
	glDeleteShader(FragmentShader);

	glDeleteProgram(Program);

	SetDefaults();
}

inline bool CShaderProgram::Load(char *VertexShaderFileName, char *FragmentShaderFileName)
{
	if(UniformLocations || VertexShader || FragmentShader || Program)
	{
		Delete();
	}

	bool Error = false;

	Error |= ((VertexShader = LoadShader(GL_VERTEX_SHADER, VertexShaderFileName)) == 0);

	Error |= ((FragmentShader = LoadShader(GL_FRAGMENT_SHADER, FragmentShaderFileName)) == 0);

	if(Error)
	{
		Delete();
		return false;
	}

	Program = glCreateProgram();
	glAttachShader(Program, VertexShader);
	glAttachShader(Program, FragmentShader);
	glLinkProgram(Program);

	int Param = 0;
	glGetProgramiv(Program, GL_LINK_STATUS, &Param);

	if(Param == GL_FALSE)
	{
		//ErrorLog.Append("Error linking program (%s, %s)!\r\n", VertexShaderFileName, FragmentShaderFileName);
		std::cerr<<"CShaderProgram: Error linking one or more of the shader programs"<<std::endl;
		int InfoLogLength = 0;
		glGetProgramiv(Program, GL_INFO_LOG_LENGTH, &InfoLogLength);
	
		if(InfoLogLength > 0)
		{
			char *InfoLog = new char[InfoLogLength];
			int CharsWritten  = 0;
			glGetProgramInfoLog(Program, InfoLogLength, &CharsWritten, InfoLog);
			//ErrorLog.Append(InfoLog);
			delete [] InfoLog;
		}

		Delete();

		return false;
	}

	return true;
}

inline GLuint CShaderProgram::LoadShader(GLenum Type, char *ShaderFileName)
{
	std::string FileName = ShaderFileName;

	FILE *File;

	if(fopen_s(&File, FileName.c_str(), "rb") != 0)
	{
		//ErrorLog.Append("Error loading file " + FileName + "!\r\n");
		std::cerr<<"CShaderProgram: Error loading "<<FileName <<std::endl;
		return 0;
	}

	fseek(File, 0, SEEK_END);
	long Size = ftell(File);
	fseek(File, 0, SEEK_SET);
	char *Source = new char[Size + 1];
	fread(Source, 1, Size, File);
	fclose(File);
	Source[Size] = 0;

	GLuint Shader;

	Shader = glCreateShader(Type);
	glShaderSource(Shader, 1, (const char**)&Source, NULL);
	delete [] Source;
	glCompileShader(Shader);

	int Param = 0;
	glGetShaderiv(Shader, GL_COMPILE_STATUS, &Param);

	if(Param == GL_FALSE)
	{
		//ErrorLog.Append("Error compiling shader %s!\r\n", ShaderFileName);
		std::cerr<<"CShaderProgram: Error compiling shader "<< ShaderFileName <<std::endl;

		int InfoLogLength = 0;
		glGetShaderiv(Shader, GL_INFO_LOG_LENGTH, &InfoLogLength);
	
		if(InfoLogLength > 0)
		{
			char *InfoLog = new char[InfoLogLength];
			int CharsWritten  = 0;
			glGetShaderInfoLog(Shader, InfoLogLength, &CharsWritten, InfoLog);
			//ErrorLog.Append(InfoLog);
			delete [] InfoLog;
		}

		glDeleteShader(Shader);

		return 0;
	}

	return Shader;
}

inline void CShaderProgram::SetDefaults()
{
	UniformLocations = NULL;
	VertexShader = 0;
	FragmentShader = 0;
	Program = 0;
}



/// Save this just in case
/*
class CShaderProgram
{
public:
	GLuint *UniformLocations;

protected:
	GLuint VertexShader, FragmentShader, Program;

public:
	CShaderProgram();
	~CShaderProgram();

	operator GLuint ();

	void Delete();
	bool Load(char *VertexShaderFileName, char *FragmentShaderFileName);

protected:
	GLuint LoadShader(GLenum Type, char *ShaderFileName);
	void SetDefaults();
};

inline CShaderProgram::CShaderProgram()
{
	SetDefaults();
}

inline CShaderProgram::~CShaderProgram()
{
}

inline CShaderProgram::operator GLuint ()
{
	return Program;
}

inline void CShaderProgram::Delete()
{
	delete [] UniformLocations;

	glDetachShader(Program, VertexShader);
	glDetachShader(Program, FragmentShader);

	glDeleteShader(VertexShader);
	glDeleteShader(FragmentShader);

	glDeleteProgram(Program);

	SetDefaults();
}

inline bool CShaderProgram::Load(char *VertexShaderFileName, char *FragmentShaderFileName)
{
	if(UniformLocations || VertexShader || FragmentShader || Program)
	{
		Delete();
	}

	bool Error = false;

	Error |= ((VertexShader = LoadShader(GL_VERTEX_SHADER, VertexShaderFileName)) == 0);

	Error |= ((FragmentShader = LoadShader(GL_FRAGMENT_SHADER, FragmentShaderFileName)) == 0);

	if(Error)
	{
		Delete();
		return false;
	}

	Program = glCreateProgram();
	glAttachShader(Program, VertexShader);
	glAttachShader(Program, FragmentShader);
	glLinkProgram(Program);

	int Param = 0;
	glGetProgramiv(Program, GL_LINK_STATUS, &Param);

	if(Param == GL_FALSE)
	{
		//ErrorLog.Append("Error linking program (%s, %s)!\r\n", VertexShaderFileName, FragmentShaderFileName);
		std::cerr<<"CShaderProgram: Error linking one or more of the shader programs"<<std::endl;
		int InfoLogLength = 0;
		glGetProgramiv(Program, GL_INFO_LOG_LENGTH, &InfoLogLength);
	
		if(InfoLogLength > 0)
		{
			char *InfoLog = new char[InfoLogLength];
			int CharsWritten  = 0;
			glGetProgramInfoLog(Program, InfoLogLength, &CharsWritten, InfoLog);
			//ErrorLog.Append(InfoLog);
			delete [] InfoLog;
		}

		Delete();

		return false;
	}

	return true;
}

inline GLuint CShaderProgram::LoadShader(GLenum Type, char *ShaderFileName)
{
	CString FileName = ModuleDirectory + ShaderFileName;

	FILE *File;

	if(fopen_s(&File, FileName, "rb") != 0)
	{
		//ErrorLog.Append("Error loading file " + FileName + "!\r\n");
		std::cerr<<"CShaderProgram: Error loading "<<FileName <<std::endl;
		return 0;
	}

	fseek(File, 0, SEEK_END);
	long Size = ftell(File);
	fseek(File, 0, SEEK_SET);
	char *Source = new char[Size + 1];
	fread(Source, 1, Size, File);
	fclose(File);
	Source[Size] = 0;

	GLuint Shader;

	Shader = glCreateShader(Type);
	glShaderSource(Shader, 1, (const char**)&Source, NULL);
	delete [] Source;
	glCompileShader(Shader);

	int Param = 0;
	glGetShaderiv(Shader, GL_COMPILE_STATUS, &Param);

	if(Param == GL_FALSE)
	{
		//ErrorLog.Append("Error compiling shader %s!\r\n", ShaderFileName);
		std::cerr<<"CShaderProgram: Error compiling shader "<< ShaderFileName <<std::endl;

		int InfoLogLength = 0;
		glGetShaderiv(Shader, GL_INFO_LOG_LENGTH, &InfoLogLength);
	
		if(InfoLogLength > 0)
		{
			char *InfoLog = new char[InfoLogLength];
			int CharsWritten  = 0;
			glGetShaderInfoLog(Shader, InfoLogLength, &CharsWritten, InfoLog);
			//ErrorLog.Append(InfoLog);
			delete [] InfoLog;
		}

		glDeleteShader(Shader);

		return 0;
	}

	return Shader;
}

inline void CShaderProgram::SetDefaults()
{
	UniformLocations = NULL;
	VertexShader = 0;
	FragmentShader = 0;
	Program = 0;
}
*/