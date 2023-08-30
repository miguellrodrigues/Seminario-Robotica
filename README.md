# Curso de Robótica - Seminário

Este projeto faz parte do curso de Robótica do Cefet-MG

## Começando

Essas instruções fornecerão uma cópia do projeto em funcionamento em sua máquina local para fins de desenvolvimento e teste.

### Pré-requisitos

CoppeliaSim e um compilador Java (Kotlin)

``` 
  Use a dll referente ao seu sistema operacional
```

<p>
    Faz se necessário a alteração da localização da DLL no arquivo <u>coppelia/remoteApi</u> linha 704
</p>

``` java

  static {
      System.load("Path da dll")
  }
```

### Instalando

Clone o repositório

<ul>
  
  <li>Abra o arquivo <u>seminario.ttt</u> utilizando o CoppeliaSim</li>
  <li>Execute o arquivo <u>com/miguel/Main.kt</u></li>
  
</ul>


### Autores


| [<img src="https://avatars1.githubusercontent.com/miguellrodrigues" width="115"><br><sub>@Miguel</sub>](https://github.com/miguellrodrigues)
|:-:|:-:|
