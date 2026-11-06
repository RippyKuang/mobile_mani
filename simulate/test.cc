/**
 * file: posix_ipc_ringbuffer.c
 * 编译命令: gcc -Wall -Wextra -O2 posix_ipc_ringbuffer.c -o posix_ipc_ringbuffer -lpthread -lrt
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <sys/wait.h>
#include <errno.h>

#define SHM_NAME       "/ipc_ringbuffer_master"
#define QUEUE_CAPACITY 8
#define MSG_PAYLOAD    64

/* 队列元素结构体 */
typedef struct {
    int sender_pid;
    int seq_id;
    char text[MSG_PAYLOAD];
} QueueMessage;

/* 共享内存内嵌自包含核心结构体 */
typedef struct {
    /* 嵌入式同步原语 (pshared = 1) */
    sem_t sem_mutex; /* 互斥保护游标更新 */
    sem_t sem_empty; /* 空余槽位计数 */
    sem_t sem_full;  /* 生产就绪数据计数 */

    /* 环形缓冲区指针与数据槽位 */
    int head;        /* 读索引 (Consumer) */
    int tail;        /* 写索引 (Producer) */
    QueueMessage slots[QUEUE_CAPACITY];
} SharedQueue;

/* 错误处理退出包装 */
static void check_error(int ret, const char *msg) {
    if (ret == -1) {
        perror(msg);
        exit(EXIT_FAILURE);
    }
}

int main(void) {
    /* 1. 确保清理遗留对象 */
    shm_unlink(SHM_NAME);

    /* 2. 创建并打开 POSIX 共享内存 */
    int fd = shm_open(SHM_NAME, O_CREAT | O_EXCL | O_RDWR, 0666);
    check_error(fd, "shm_open failed");

    /* 3. 设置共享内存大小 (必须执行以避免 SIGBUS) */
    int tr_ret = ftruncate(fd, sizeof(SharedQueue));
    check_error(tr_ret, "ftruncate failed");

    /* 4. 内存映射 */
    SharedQueue *q = (SharedQueue *)mmap(NULL, sizeof(SharedQueue),
                                         PROT_READ | PROT_WRITE,
                                         MAP_SHARED, fd, 0);
    close(fd); /* 建立映射后立即关闭描述符 */
    if (q == MAP_FAILED) {
        perror("mmap failed");
        shm_unlink(SHM_NAME);
        exit(EXIT_FAILURE);
    }

    /* 5. 初始化自包含队列及嵌入式信号量 (pshared = 1 表示跨进程共享) */
    q->head = 0;
    q->tail = 0;
    check_error(sem_init(&q->sem_mutex, 1, 1), "sem_init mutex failed");
    check_error(sem_init(&q->sem_empty, 1, QUEUE_CAPACITY), "sem_init empty failed");
    check_error(sem_init(&q->sem_full,  1, 0), "sem_init full failed");

    /* 6. 创建并发子进程 */
    pid_t pid = fork();
    check_error(pid, "fork failed");

    if (pid == 0) {
        /* 子进程: 消费者进程 */
        printf("[Consumer PID %d] 消费者就绪，准备消费消息...\n", getpid());
        for (int i = 0; i < 5; i++) {
            /* P 操作: 等待有数据，获取互斥锁 */
            sem_wait(&q->sem_full);
            sem_wait(&q->sem_mutex);

            /* 读取数据 */
            QueueMessage msg = q->slots[q->head];
            q->head = (q->head + 1) % QUEUE_CAPACITY;

            /* V 操作: 释放互斥锁，增加空槽位 */
            sem_post(&q->sem_mutex);
            sem_post(&q->sem_empty);

            printf("  -> [Consumer PID %d] 消费第 %d 条数据 [Seq: %d, 来源 PID: %d, 内容: '%s']\n",
                   getpid(), i + 1, msg.seq_id, msg.sender_pid, msg.text);
            fflush(stdout);
            usleep(30000); /* 模拟业务消费耗时 */
        }

        munmap(q, sizeof(SharedQueue));
        exit(EXIT_SUCCESS);
    } else {
        /* 父进程: 生产者进程 */
        printf("[Producer PID %d] 生产者启动，写入数据流...\n", getpid());
        for (int i = 0; i < 5; i++) {
            /* P 操作: 等待空闲槽位，获取互斥锁 */
            sem_wait(&q->sem_empty);
            sem_wait(&q->sem_mutex);

            /* 填充数据 */
            q->slots[q->tail].sender_pid = getpid();
            q->slots[q->tail].seq_id = i + 1;
            snprintf(q->slots[q->tail].text, MSG_PAYLOAD, "Heartbeat payload data %d", i + 1);
            q->tail = (q->tail + 1) % QUEUE_CAPACITY;

            /* V 操作: 释放互斥锁，通知新就绪数据 */
            sem_post(&q->sem_mutex);
            sem_post(&q->sem_full);

            printf("[Producer PID %d] 投递第 %d 条消息成功\n", getpid(), i + 1);
            fflush(stdout);
            usleep(10000);
        }

        /* 等待消费子进程结束 */
        int status;
        waitpid(pid, &status, 0);

        /* 7. 优雅反初始化与销毁资源 */
        sem_destroy(&q->sem_mutex);
        sem_destroy(&q->sem_empty);
        sem_destroy(&q->sem_full);
        munmap(q, sizeof(SharedQueue));
        shm_unlink(SHM_NAME);
        printf("[Clean] POSIX 环形缓冲区及信号量安全销毁完成。\n");
    }

    return EXIT_SUCCESS;
}